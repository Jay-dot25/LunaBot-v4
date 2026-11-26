# LunaBot P9-03 Dashboard Tools

This Foxglove extension measures the `/lunabot/camera/image_raw` panel's
receive-to-paint delay and displays the camera stream beside the metrics. It
subscribes to `/clock` to estimate source-stamp-to-paint wall age while the
simulation runs below real time. Use the export button to save up to 2,000
samples as CSV.

It also registers six **Topic health** panels, one for each view in
`config/dashboard/contracts.json`. Each panel subscribes to its view's source
topics and `/lunabot/dashboard/telemetry`, and shows adapter freshness, source
age, receive age, and client receive-to-render timing. Source-to-render timing
is an estimate based on source stamps and `/clock`; it is unavailable when a
message has no ROS stamp. The panels are read-only and do not publish or call
services.

## Build

Requires Node.js and pnpm. From this directory:

```sh
pnpm install
pnpm build
pnpm package
```

The package command creates a `.foxe` extension archive. Install it in the
Foxglove client, then add the six **Topic health — …** panels to their matching
tabs. Add the **camera-render-probe** panel only for camera timing diagnostics.

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
