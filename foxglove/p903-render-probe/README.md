# LunaBot Camera Render Probe

This Foxglove extension measures the `/lunabot/camera/image_raw` panel's
receive-to-paint delay and displays the camera stream beside the metrics. It
subscribes to `/clock` to estimate source-stamp-to-paint wall age while the
simulation runs below real time. Use the export button to save up to 2,000
samples as CSV.

## Build

Requires Node.js and pnpm. From this directory:

```sh
pnpm install
pnpm build
pnpm package
```

The package command creates a `.foxe` extension archive. Install it in a
Foxglove desktop or web organization that permits local extensions, then add
the **Camera render probe** panel and leave it open during the benchmark.

## Metrics and limits

- **Painted FPS** counts frames painted in the last five seconds.
- **Receive → paint** is measured with the browser's monotonic clock.
- **Source → paint wall age** maps the ROS source stamp onto wall time using
  recent `/clock` samples and their observed simulation rate. Treat it as an
  estimate, especially during clock resets or abrupt rate changes.
- **Coalesced** counts frames combined by Foxglove or superseded before the
  panel painted them. **Estimated source gaps** compares source stamps with an
  expected 20 Hz simulation cadence; it is not a transport-level drop counter.

The panel converts common ROS `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, and
`8UC1` image encodings to canvas pixels. These measurements characterize this
probe panel and its browser environment; they do not directly measure the
built-in Foxglove Image panel. Keep the contract dashboard open alongside the
probe and report this distinction with any P9-03 benchmark results.
