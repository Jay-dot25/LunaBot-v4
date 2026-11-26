import { MessageEvent, PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

const TELEMETRY_TOPIC = "/lunabot/dashboard/telemetry";
const CLOCK_TOPIC = "/clock";
const SAMPLE_LIMIT = 300;

type TopicSpec = { topic: string; type: string };
type RosStamp = { sec?: number; nanosec?: number; nsec?: number };
type StampedMessage = { header?: { stamp?: RosStamp; frame_id?: string } };
type RosClock = { clock?: RosStamp };
type AdapterTopic = {
  topic: string;
  type: string;
  state: string;
  state_reason?: string;
  source_stamp_seconds?: number | null;
  source_age_seconds?: number | null;
  receive_age_seconds?: number | null;
  frame_id?: string | null;
  expected_frame?: string | null;
  source_frame_ids?: string[];
  source_nodes?: string[];
  subscription_error?: string | null;
};
type AdapterMessage = { data?: string };
type ClockSample = { simSeconds: number; perfMs: number; simRate: number };
type TopicStats = {
  received: number;
  lastReceivePerfMs: number | null;
  receiveToRenderMs: number[];
  sourceToRenderMs: number[];
};
type PendingBatch = { events: MessageEvent[]; receivePerfMs: number; done: () => void };

export type TopicHealthGroup = {
  title: string;
  topics: TopicSpec[];
};

function stampSeconds(stamp?: RosStamp): number | null {
  const nanos = stamp?.nanosec ?? stamp?.nsec;
  if (stamp == null || !Number.isFinite(stamp.sec) || !Number.isFinite(nanos)) { return null; }
  return (stamp.sec ?? 0) + (nanos ?? 0) / 1_000_000_000;
}

function percentile(values: number[], p: number): number | null {
  if (values.length === 0) { return null; }
  const sorted = [...values].sort((a, b) => a - b);
  return sorted[Math.max(0, Math.min(sorted.length - 1, Math.ceil(p * sorted.length) - 1))] ?? null;
}

function parseAdapter(message: unknown): Map<string, AdapterTopic> {
  const text = (message as AdapterMessage | undefined)?.data;
  if (!text) { return new Map(); }
  try {
    const parsed = JSON.parse(text) as { topics?: AdapterTopic[] };
    return new Map((parsed.topics ?? []).map((row) => [row.topic, row]));
  } catch {
    return new Map();
  }
}

function formatAge(seconds: number | null | undefined): string {
  return seconds == null || !Number.isFinite(seconds) ? "n/a" : `${(seconds * 1_000).toFixed(0)} ms`;
}

function TopicHealthPanel({ context, group }: { context: PanelExtensionContext; group: TopicHealthGroup }): ReactElement {
  const pendingRef = useRef<PendingBatch | null>(null);
  const clockRef = useRef<ClockSample | null>(null);
  const previousClockRef = useRef<{ simSeconds: number; perfMs: number } | null>(null);
  const [metadata, setMetadata] = useState<Map<string, AdapterTopic>>(new Map());
  const [stats, setStats] = useState<Map<string, TopicStats>>(new Map());
  const [simRate, setSimRate] = useState<number | null>(null);
  const [renderCount, setRenderCount] = useState(0);
  const [pendingBatch, setPendingBatch] = useState<PendingBatch | null>(null);

  useLayoutEffect(() => {
    let completePending: (() => void) | null = null;
    const wanted = new Set(group.topics.map((item) => item.topic));
    wanted.add(TELEMETRY_TOPIC);
    wanted.add(CLOCK_TOPIC);

    context.onRender = (renderState, done) => {
      const frameEvents = (renderState.currentFrame ?? []).filter((event) => wanted.has(event.topic));
      if (frameEvents.length === 0) { done(); return; }
      const receivedAt = performance.now();
      for (const event of frameEvents) {
        if (event.topic === CLOCK_TOPIC) {
          const simSeconds = stampSeconds((event.message as RosClock).clock);
          if (simSeconds == null) { continue; }
          const previous = previousClockRef.current;
          let rate = clockRef.current?.simRate ?? 0;
          if (previous) {
            const wallDelta = (receivedAt - previous.perfMs) / 1_000;
            const simDelta = simSeconds - previous.simSeconds;
            if (wallDelta > 0 && simDelta >= 0) { rate = simDelta / wallDelta; }
          }
          clockRef.current = { simSeconds, perfMs: receivedAt, simRate: rate };
          previousClockRef.current = { simSeconds, perfMs: receivedAt };
          setSimRate(rate);
        } else if (event.topic === TELEMETRY_TOPIC) {
          const nextMetadata = parseAdapter(event.message);
          if (nextMetadata.size > 0) { setMetadata(nextMetadata); }
        }
      }

      let completed = false;
      const finish = () => {
        if (completed) { return; }
        completed = true;
        done();
        completePending = null;
      };
      if (completePending) { completePending(); }
      const next = { events: frameEvents, receivePerfMs: receivedAt, done: finish };
      pendingRef.current = next;
      completePending = finish;
      setPendingBatch(next);
    };
    context.watch("currentFrame");
    context.subscribe([
      ...group.topics.map(({ topic }) => ({ topic })),
      { topic: TELEMETRY_TOPIC },
      { topic: CLOCK_TOPIC },
    ]);
    return () => {
      completePending?.();
      pendingRef.current = null;
      context.subscribe([]);
    };
  }, [context, group]);

  useLayoutEffect(() => {
    if (!pendingBatch) { return; }
    let firstFrame = 0;
    const secondFrame = requestAnimationFrame(() => {
      firstFrame = requestAnimationFrame(() => {
        const renderedAt = performance.now();
        setStats((previous) => {
          const next = new Map(previous);
          for (const event of pendingBatch.events) {
            if (event.topic === TELEMETRY_TOPIC || event.topic === CLOCK_TOPIC) { continue; }
            const old = next.get(event.topic) ?? {
              received: 0, lastReceivePerfMs: null, receiveToRenderMs: [], sourceToRenderMs: [],
            };
            const messageStamp = stampSeconds((event.message as StampedMessage).header?.stamp);
            const clock = clockRef.current;
            let sourceToRenderMs: number | null = null;
            if (messageStamp != null && clock != null && clock.simRate > 0.01) {
              const sourcePerfMs = clock.perfMs - ((clock.simSeconds - messageStamp) / clock.simRate) * 1_000;
              sourceToRenderMs = renderedAt - sourcePerfMs;
            }
            const receiveToRender = [...old.receiveToRenderMs, renderedAt - pendingBatch.receivePerfMs].slice(-SAMPLE_LIMIT);
            const sourceToRender = sourceToRenderMs == null
              ? old.sourceToRenderMs
              : [...old.sourceToRenderMs, sourceToRenderMs].slice(-SAMPLE_LIMIT);
            next.set(event.topic, {
              received: old.received + 1,
              lastReceivePerfMs: pendingBatch.receivePerfMs,
              receiveToRenderMs: receiveToRender,
              sourceToRenderMs: sourceToRender,
            });
          }
          return next;
        });
        setRenderCount((value) => value + pendingBatch.events.length);
        if (pendingRef.current === pendingBatch) { pendingRef.current = null; }
        pendingBatch.done();
      });
    });
    return () => {
      cancelAnimationFrame(secondFrame);
      if (firstFrame !== 0) { cancelAnimationFrame(firstFrame); }
    };
  }, [pendingBatch]);

  const formatLatency = (values: number[]) => {
    const p95 = percentile(values, 0.95);
    return p95 == null ? "n/a" : `${p95.toFixed(1)} ms`;
  };

  return <div style={{ height: "100%", overflow: "auto", background: "#111", color: "#eee", padding: 12, font: "12px sans-serif" }}>
    <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline", gap: 8, flexWrap: "wrap" }}>
      <strong>{group.title} — live topic contract</strong>
      <span style={{ color: "#aaa" }}>Panel renders {renderCount} source updates · simulation rate {simRate == null ? "n/a" : `${simRate.toFixed(3)}×`}</span>
    </div>
    <div style={{ color: "#aaa", margin: "6px 0 10px" }}>
      Receive → render p95 is measured inside this panel after a paint opportunity. Source age and freshness come from the read-only adapter; source → render uses `/clock` and is an estimate.
    </div>
    <div style={{ display: "grid", gridTemplateColumns: "minmax(260px, 2fr) repeat(5, minmax(100px, 1fr))", gap: 4, borderBottom: "1px solid #444", padding: "4px 2px", color: "#aaa", fontWeight: 600 }}>
      <span>Topic</span><span>State / reason</span><span>Source age</span><span>Receive age</span><span>Receive → render p95</span><span>Source → render p95*</span>
    </div>
    {group.topics.map(({ topic, type }) => {
      const row = metadata.get(topic);
      const topicStats = stats.get(topic);
      const stateColor = row?.state === "fresh" ? "#6dd17c" : row?.state === "stale" ? "#ffbd5a" : "#9ca3af";
      return <div key={topic} style={{ display: "grid", gridTemplateColumns: "minmax(260px, 2fr) repeat(5, minmax(100px, 1fr))", gap: 4, padding: "6px 2px", borderBottom: "1px solid #272727", alignItems: "start" }}>
        <span title={`${type}${row?.frame_id ? ` · frame ${row.frame_id}` : ""}`} style={{ color: "#e8e8e8", overflowWrap: "anywhere" }}>{topic}</span>
        <span style={{ color: stateColor }}>{row?.state ?? "unknown"}{row?.state_reason ? ` · ${row.state_reason}` : " · no metadata"}</span>
        <span>{formatAge(row?.source_age_seconds)}</span>
        <span>{formatAge(row?.receive_age_seconds)}</span>
        <span>{formatLatency(topicStats?.receiveToRenderMs ?? [])}</span>
        <span>{formatLatency(topicStats?.sourceToRenderMs ?? [])}</span>
      </div>;
    })}
    <div style={{ marginTop: 10, color: "#aaa" }}>
      * Estimated on each received ROS-stamped message. Rows without a source stamp show n/a. Missing sources remain unknown.
    </div>
  </div>;
}

export function initTopicHealthPanel(context: PanelExtensionContext, group: TopicHealthGroup): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TopicHealthPanel context={context} group={group} />);
  return () => { root.unmount(); };
}
