import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

const IMAGE_TOPIC = "/lunabot/camera/image_raw";
const CLOCK_TOPIC = "/clock";
const EXPECTED_SIM_RATE_HZ = 20;
const SAMPLE_LIMIT = 2_000;

type RosStamp = { sec?: number; nanosec?: number; nsec?: number };
type RosImage = {
  header?: { stamp?: RosStamp; frame_id?: string };
  width?: number;
  height?: number;
  step?: number;
  encoding?: string;
  data?: Uint8Array | number[];
};
type RosClock = { clock?: RosStamp };
type ClockSample = { simSeconds: number; perfMs: number; simRate: number };
type FrameSample = {
  sourceStampSeconds: number | null;
  receivePerfMs: number;
  renderPerfMs: number;
  receiveToRenderMs: number;
  sourceToRenderWallMs: number | null;
  width: number;
  height: number;
  encoding: string;
};
type PendingFrame = { message: RosImage; receivedAtMs: number; done: () => void };

function stampSeconds(stamp?: RosStamp): number | null {
  const nanoseconds = stamp?.nanosec ?? stamp?.nsec;
  if (stamp == null || !Number.isFinite(stamp.sec) || !Number.isFinite(nanoseconds)) {return null;}
  return (stamp.sec ?? 0) + (nanoseconds ?? 0) / 1_000_000_000;
}

function percentile(values: number[], p: number): number | null {
  if (values.length === 0) {return null;}
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.min(sorted.length - 1, Math.ceil(p * sorted.length) - 1))] ?? null;
}

function toImageData(message: RosImage): ImageData | null {
  const width = message.width ?? 0;
  const height = message.height ?? 0;
  const step = message.step ?? 0;
  const encoding = (message.encoding ?? "").toLowerCase();
  if (width === 0 || height === 0 || step === 0 || message.data == null) {return null;}
  const source = message.data instanceof Uint8Array ? message.data : Uint8Array.from(message.data);
  const bytesPerPixel = encoding === "rgba8" || encoding === "bgra8" ? 4
    : encoding === "rgb8" || encoding === "bgr8" ? 3
    : encoding === "mono8" || encoding === "8uc1" ? 1 : 0;
  if (bytesPerPixel === 0 || step < width * bytesPerPixel || source.length < step * height) {return null;}
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      const s = y * step + x * bytesPerPixel;
      const d = (y * width + x) * 4;
      if (bytesPerPixel === 1) {
        const gray = source[s] ?? 0;
        rgba[d] = gray; rgba[d + 1] = gray; rgba[d + 2] = gray; rgba[d + 3] = 255;
      } else if (encoding === "rgb8" || encoding === "rgba8") {
        rgba[d] = source[s] ?? 0; rgba[d + 1] = source[s + 1] ?? 0;
        rgba[d + 2] = source[s + 2] ?? 0; rgba[d + 3] = bytesPerPixel === 4 ? source[s + 3] ?? 255 : 255;
      } else {
        rgba[d] = source[s + 2] ?? 0; rgba[d + 1] = source[s + 1] ?? 0;
        rgba[d + 2] = source[s] ?? 0; rgba[d + 3] = bytesPerPixel === 4 ? source[s + 3] ?? 255 : 255;
      }
    }
  }
  return new ImageData(rgba, width, height);
}

function eventsFor(frame: readonly MessageEvent[] | undefined, topic: string): MessageEvent[] {
  return (frame ?? []).filter((event) => event.topic === topic);
}

function CameraRenderProbe({ context }: { context: PanelExtensionContext }): ReactElement {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const clockRef = useRef<ClockSample | null>(null);
  const priorClockRef = useRef<{ simSeconds: number; perfMs: number } | null>(null);
  const lastSourceStampRef = useRef<number | null>(null);
  const pendingRef = useRef<PendingFrame | null>(null);
  const [pending, setPending] = useState<PendingFrame | null>(null);
  const [samples, setSamples] = useState<FrameSample[]>([]);
  const [received, setReceived] = useState(0);
  const [coalesced, setCoalesced] = useState(0);
  const [sourceGaps, setSourceGaps] = useState(0);
  const [error, setError] = useState<string | null>(null);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      let now = performance.now();
      for (const event of eventsFor(renderState.currentFrame, CLOCK_TOPIC)) {
        const simSeconds = stampSeconds((event.message as RosClock).clock);
        if (simSeconds == null) {continue;}
        const previous = priorClockRef.current;
        let simRate = clockRef.current?.simRate ?? 0;
        if (previous) {
          const wallDelta = (now - previous.perfMs) / 1_000;
          const simDelta = simSeconds - previous.simSeconds;
          if (wallDelta > 0 && simDelta >= 0) {simRate = simDelta / wallDelta;}
        }
        clockRef.current = { simSeconds, perfMs: now, simRate };
        priorClockRef.current = { simSeconds, perfMs: now };
      }

      const images = eventsFor(renderState.currentFrame, IMAGE_TOPIC);
      if (images.length === 0) { done(); return; }
      const latest = images[images.length - 1];
      if (!latest) { done(); return; }
      now = performance.now();
      let completed = false;
      const finish = () => {
        if (completed) {return;}
        completed = true;
        done();
      };
      const next = { message: latest.message as RosImage, receivedAtMs: now, done: finish };
      setReceived((count) => count + images.length);
      if (images.length > 1) {setCoalesced((count) => count + images.length - 1);}
      const sourceStamp = stampSeconds(next.message.header?.stamp);
      const priorStamp = lastSourceStampRef.current;
      if (sourceStamp != null && priorStamp != null && sourceStamp - priorStamp > 1.5 / EXPECTED_SIM_RATE_HZ) {
        setSourceGaps((count) => count + Math.max(1, Math.round((sourceStamp - priorStamp) * EXPECTED_SIM_RATE_HZ) - 1));
      }
      if (sourceStamp != null) {lastSourceStampRef.current = sourceStamp;}
      // If another batch arrives before React paints this frame, finish the
      // earlier Foxglove render callback so the data pipeline cannot stall.
      if (pendingRef.current) {
        pendingRef.current.done();
        setCoalesced((count) => count + 1);
      }
      pendingRef.current = next;
      setPending(next);
    };
    context.watch("currentFrame");
    context.subscribe([{ topic: IMAGE_TOPIC }, { topic: CLOCK_TOPIC }]);
    return () => {
      pendingRef.current?.done();
      pendingRef.current = null;
      context.subscribe([]);
    };
  }, [context]);

  useLayoutEffect(() => {
    if (!pending) {return;}
    const canvas = canvasRef.current;
    const message = pending.message;
    const width = message.width ?? 0;
    const height = message.height ?? 0;
    const image = toImageData(message);
    if (!canvas || !image) {
      setError(`Unsupported or invalid image encoding: ${message.encoding ?? "unknown"}`);
    } else {
      setError(null);
      canvas.width = width;
      canvas.height = height;
      canvas.getContext("2d", { alpha: false })?.putImageData(image, 0, 0);
    }
    // First rAF is before paint; the second runs after the canvas had a paint opportunity.
    requestAnimationFrame(() => requestAnimationFrame(() => {
      const renderPerfMs = performance.now();
      const sourceStampSeconds = stampSeconds(message.header?.stamp);
      const clock = clockRef.current;
      const sourcePerfMs = sourceStampSeconds != null && clock != null && clock.simRate > 0.01
        ? clock.perfMs - ((clock.simSeconds - sourceStampSeconds) / clock.simRate) * 1_000
        : null;
      const row: FrameSample = {
        sourceStampSeconds,
        receivePerfMs: pending.receivedAtMs,
        renderPerfMs,
        receiveToRenderMs: renderPerfMs - pending.receivedAtMs,
        sourceToRenderWallMs: sourcePerfMs == null ? null : renderPerfMs - sourcePerfMs,
        width, height, encoding: message.encoding ?? "unknown",
      };
      setSamples((previous) => [...previous, row].slice(-SAMPLE_LIMIT));
      if (pendingRef.current === pending) {pendingRef.current = null;}
      pending.done();
    }));
  }, [pending]);

  const now = performance.now();
  const lastFiveSeconds = samples.filter((sample) => now - sample.renderPerfMs <= 5_000);
  const firstRecentSample = lastFiveSeconds[0];
  const lastRecentSample = lastFiveSeconds[lastFiveSeconds.length - 1];
  const fps = lastFiveSeconds.length < 2 ? null : (lastFiveSeconds.length - 1) * 1_000 /
    ((lastRecentSample?.renderPerfMs ?? now) - (firstRecentSample?.renderPerfMs ?? now));
  const receiveToRender = samples.map((sample) => sample.receiveToRenderMs);
  const sourceToRender = samples.flatMap((sample) => sample.sourceToRenderWallMs == null ? [] : [sample.sourceToRenderWallMs]);
  const fmt = (value: number | null) => value == null ? "n/a" : `${value.toFixed(1)} ms`;
  const exportCsv = () => {
    const content = [
      "source_stamp_seconds,receive_perf_ms,render_perf_ms,receive_to_render_ms,source_to_render_wall_ms,width,height,encoding",
      ...samples.map((sample) => [sample.sourceStampSeconds ?? "", sample.receivePerfMs.toFixed(3),
        sample.renderPerfMs.toFixed(3), sample.receiveToRenderMs.toFixed(3),
        sample.sourceToRenderWallMs?.toFixed(3) ?? "", sample.width, sample.height, sample.encoding].join(",")),
    ].join("\n");
    const url = URL.createObjectURL(new Blob([content], { type: "text/csv" }));
    const link = document.createElement("a");
    link.href = url;
    link.download = "lunabot-camera-render-samples.csv";
    link.click();
    URL.revokeObjectURL(url);
  };

  return <div style={{ height: "100%", overflow: "auto", background: "#111", color: "#eee", padding: 12, font: "12px sans-serif" }}>
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
      <strong>Camera render probe — {IMAGE_TOPIC}</strong>
      <button onClick={exportCsv} disabled={samples.length === 0}>Export CSV</button>
    </div>
    <div style={{ display: "grid", gridTemplateColumns: "repeat(3, minmax(120px, 1fr))", gap: 8, margin: "10px 0" }}>
      <Metric label="Painted FPS (5s)" value={fps == null ? "n/a" : fps.toFixed(1)} />
      <Metric label="Receive → paint p50 / p95" value={`${fmt(percentile(receiveToRender, 0.5))} / ${fmt(percentile(receiveToRender, 0.95))}`} />
      <Metric label="Source → paint wall p50 / p95" value={`${fmt(percentile(sourceToRender, 0.5))} / ${fmt(percentile(sourceToRender, 0.95))}`} />
      <Metric label="Received frames" value={String(received)} />
      <Metric label="Coalesced in client batch" value={String(coalesced)} />
      <Metric label="Estimated source gaps" value={String(sourceGaps)} />
    </div>
    <div style={{ minHeight: 160, display: "grid", placeItems: "center", background: "#000" }}>
      <canvas ref={canvasRef} style={{ maxWidth: "100%", maxHeight: "55vh", objectFit: "contain" }} />
      {samples.length === 0 && <span>Waiting for camera frames and /clock…</span>}
    </div>
    <div style={{ color: error ? "#ff8b8b" : "#aaa", marginTop: 6 }}>
      {error ?? `Rendered ${samples.length} samples. Source-time mapping uses /clock; ${clockRef.current?.simRate.toFixed(3) ?? "n/a"}× real-time.`}
    </div>
  </div>;
}

function Metric({ label, value }: { label: string; value: string }): ReactElement {
  return <div style={{ background: "#202020", padding: 8 }}><div style={{ color: "#aaa" }}>{label}</div><strong>{value}</strong></div>;
}

export function initCameraRenderProbe(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CameraRenderProbe context={context} />);
  return () => { root.unmount(); };
}
