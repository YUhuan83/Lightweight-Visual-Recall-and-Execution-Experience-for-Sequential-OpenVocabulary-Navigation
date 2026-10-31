"""Rearrange the live WildOS model image for a large RViz image pane."""
from copy import deepcopy
import cv2
import numpy as np


def rgb_target_image(message, mask_message=None, preview_width=960, max_mask_age_sec=0.5):
    """Overlay a fresh mask on a small preview of the independent camera stream."""
    output = deepcopy(message)
    if (message.encoding not in ('rgb8', 'bgr8') or message.height <= 0 or message.width <= 0
            or message.step < message.width * 3
            or len(message.data) != message.height * message.step):
        return output
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
    if mask_message is not None:
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        mask_stamp = mask_message.header.stamp.sec * 10**9 + mask_message.header.stamp.nanosec
        dims = tuple(d.size for d in mask_message.object_mask.layout.dim)
        # Delayed inference must not paint an old object onto a new scene.
        max_age = min(2., max(0., max_mask_age_sec)) * 10**9
        if (0 <= stamp - mask_stamp <= max_age
                and dims == (1, 1, message.height, message.width)
                and len(mask_message.object_mask.data) == message.height * message.width):
            pixels = pixels.copy()
            mask = np.asarray(mask_message.object_mask.data, dtype=np.uint8).reshape(dims)[0, 0] > 0
            pixels[mask] = (.6 * pixels[mask] + .4 * np.array([0, 255, 0])).astype(np.uint8)
    if 0 < preview_width < message.width:
        height = max(1, round(message.height * preview_width / message.width))
        pixels = cv2.resize(pixels, (preview_width, height), interpolation=cv2.INTER_AREA)
    output.height, output.width = map(int, pixels.shape[:2])
    output.step = output.width * 3
    output.data = np.ascontiguousarray(pixels).tobytes()
    return output


def frontier_dashboard(message):
    # WildOS visualize_model_det emits four equal rows and a 120 px colorbar.
    # Keep the model's rendered values and published header.
    if (message.encoding not in ('rgb8', 'bgr8') or message.height < 160 or message.height % 4
            or message.width <= 120 or message.step < message.width * 3
            or len(message.data) != message.height * message.step):
        return deepcopy(message)
    pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    pixels = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
    tile_width = message.width - 120
    tile_height = message.height // 4
    tiles = [pixels[i*tile_height:(i+1)*tile_height, :tile_width].copy() for i in range(4)]
    # The source's long titles are clipped at this single-camera resolution.
    # Replace only their white header band; heatmaps and camera pixels remain.
    labels = ('RGB + target mask', 'Frontier confidence',
              'Traversability confidence', 'Frontier nodes / paths')
    for tile, label in zip(tiles, labels):
        tile[:40] = 255
        size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
        cv2.putText(tile, label, (max(0, (tile_width-size[0])//2), 28),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 1, cv2.LINE_AA)
    # Frontier/traversability get the first row; RGB/object mask and projected
    # frontier paths get the second. Display layout cannot affect planning.
    canvas = np.concatenate((np.concatenate((tiles[1], tiles[2]), axis=1),
                             np.concatenate((tiles[0], tiles[3]), axis=1)), axis=0)
    # Preserve the source color legend at half vertical size without new scores.
    legend = pixels[::2, tile_width:]
    canvas = np.concatenate((canvas, legend), axis=1)
    output = deepcopy(message)
    output.height, output.width = map(int, canvas.shape[:2])
    output.step = output.width * 3
    output.data = np.ascontiguousarray(canvas).tobytes()
    return output

