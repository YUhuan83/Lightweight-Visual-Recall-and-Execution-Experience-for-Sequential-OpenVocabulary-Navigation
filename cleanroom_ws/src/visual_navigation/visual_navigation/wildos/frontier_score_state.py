"""Provenance and expiry for model scores; defaults are not observations."""

import numpy as np


def score_state(previous, scores, *, default, now, semantic, beta):
    values = np.asarray(scores, dtype=np.float32).reshape(-1).copy()
    if not values.size or not np.all(np.isfinite(values)) or not np.isfinite(now):
        return None
    values = np.clip(values, 0.0, 1.0)
    reset = (
        default
        or previous is None
        or previous.get("default", True)
        or previous["mean"].shape != values.shape
    )
    if reset:
        mean, m2, count = values.copy(), np.zeros_like(values), 0 if default else 1
    else:
        count = previous["count"] + 1
        delta = values - previous["mean"]
        mean = previous["mean"] + delta / count
        m2 = previous["m2"] + delta * (values - mean)
    std = np.sqrt(np.maximum(m2 / max(count - 1, 1), 0.0))
    safe = np.clip(mean - beta * std, 0.0 if default else 1e-4, 1.0).astype(np.float32)
    return dict(
        mean=mean,
        m2=m2,
        count=count,
        raw=values,
        safe=safe,
        default=bool(default),
        observed_at=float(now),
        semantic=float(semantic) if np.isfinite(semantic) else 0.0,
    )


def fresh_visual(state, now, maximum_age):
    return (
        state is not None
        and not state.get("default", True)
        and 0 <= now - state["observed_at"] <= maximum_age
    )
