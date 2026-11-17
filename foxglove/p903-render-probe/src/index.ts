import { ExtensionContext } from "@foxglove/extension";

import { initCameraRenderProbe } from "./CameraRenderProbe";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "camera-render-probe", initPanel: initCameraRenderProbe });
}
