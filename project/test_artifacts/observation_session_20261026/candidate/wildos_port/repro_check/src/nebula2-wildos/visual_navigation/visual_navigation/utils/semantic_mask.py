"""Filter weak semantic regions at model resolution, before image upsampling."""
import cv2
import numpy as np


def supported_mask_components(mask, minimum_cells):
    """Keep independently supported 8-connected regions in each BxQ feature mask."""
    if isinstance(minimum_cells, bool) or int(minimum_cells) != minimum_cells or minimum_cells < 0:
        raise ValueError('minimum_cells must be a non-negative integer')
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 4:
        raise ValueError('semantic mask must have shape BxQxHxW')
    if minimum_cells <= 1:
        return mask.copy()
    result = np.zeros_like(mask)
    for batch in range(mask.shape[0]):
        for query in range(mask.shape[1]):
            _, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask[batch, query].astype(np.uint8), connectivity=8)
            supported = stats[:, cv2.CC_STAT_AREA] >= minimum_cells
            supported[0] = False
            result[batch, query] = supported[labels]
    return result


def largest_mask_component(mask):
    """Return one connected candidate, avoiding centroids between instances."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError('instance mask must be two-dimensional')
    if not mask.any():
        return mask.copy()
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8)
    if count <= 1:
        return mask.copy()
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return labels == component
