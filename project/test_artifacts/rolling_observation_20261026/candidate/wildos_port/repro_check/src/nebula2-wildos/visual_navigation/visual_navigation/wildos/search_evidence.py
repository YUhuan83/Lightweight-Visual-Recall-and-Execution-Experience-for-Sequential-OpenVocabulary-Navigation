"""Convert valid detector evidence to bounded search preference, not absence truth."""
import numpy as np


def negative_search_strength(similarity, mask_threshold):
    """Return threshold-relative evidence; invalid data is not a negative view.

    A high target similarity rejected by competition/component support remains
    ambiguous. Raw cosine is not a probability; 1-cosine is not confidence.
    """
    values = np.asarray(similarity, dtype=np.float32)
    threshold = float(mask_threshold)
    if (not values.size or not np.isfinite(values).all() or not np.isfinite(threshold)
            or not -1. < threshold < 1.
            or values.min() < -1.001 or values.max() > 1.001):
        return None
    maximum = float(np.clip(values.max(), -1., 1.))
    return float(np.clip((threshold-maximum)/(threshold+1.), 0., 1.))
