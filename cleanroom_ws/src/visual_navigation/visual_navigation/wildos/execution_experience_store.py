"""Execution feedback history for scene-conditioned heading priors.

SQLite ':memory:' keeps the store within this process. Storage lifetime is
chosen by the existing configuration; this class does not change that policy.
"""

import sqlite3
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


def fuse_execution_prior(scores, prior, weight):
    """Apply bounded execution evidence only to already supported directions."""
    source = np.asarray(scores, dtype=np.float32)
    factors = np.asarray(prior, dtype=np.float32)
    if (
        source.ndim == 0
        or factors.ndim != 1
        or factors.size != source.shape[-1]
        or not np.isfinite(factors).all()
        or not np.isfinite(weight)
    ):
        return source.copy()
    blend = float(np.clip(weight, 0.0, 1.0))
    fused = source * (1.0 + blend * (np.clip(factors, 0.0, 2.0) - 1.0))
    # Query rejection/semantic suppression cannot acquire support from memory.
    return np.where(source > 1e-4, np.clip(fused, 0.0, 1.0), source).astype(np.float32)


class ExecutionExperienceStore:
    """CPU-friendly SQLite index of RADIO descriptors and heading targets."""

    def __init__(
        self,
        database_path: str,
        encoder_id: str,
        max_records: int = 4096,
        similarity_threshold: float = 0.82,
        label_strength: float = 0.8,
        map_id: str = "default",
    ):
        self.database_path = Path(database_path).expanduser()
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self.encoder_id = str(encoder_id)
        self.map_id = str(map_id)
        self.max_records = max(1, int(max_records))
        self.retrieval_candidates = min(self.max_records, 512)
        self.similarity_threshold = float(np.clip(similarity_threshold, -1.0, 1.0))
        self.label_strength = float(np.clip(label_strength, 0.0, 1.0))
        self._db = sqlite3.connect(str(self.database_path), timeout=5.0)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.execute("""CREATE TABLE IF NOT EXISTS experiences (
            id INTEGER PRIMARY KEY AUTOINCREMENT, encoder_id TEXT NOT NULL,
            map_id TEXT NOT NULL DEFAULT 'default',
            embedding BLOB NOT NULL, embedding_dim INTEGER NOT NULL,
            heading_scores BLOB NOT NULL, heading_count INTEGER NOT NULL,
            outcome INTEGER NOT NULL, created_at REAL NOT NULL)""")
        columns = {row[1] for row in self._db.execute("PRAGMA table_info(experiences)")}
        if "map_id" not in columns:
            self._db.execute(
                "ALTER TABLE experiences ADD COLUMN map_id TEXT NOT NULL DEFAULT 'default'"
            )
        if "supervised_mask" not in columns:
            self._db.execute("ALTER TABLE experiences ADD COLUMN supervised_mask BLOB")
        if "execution_multiplier" not in columns:
            # NULL keeps old [0,1] penalty records in their original semantics.
            self._db.execute(
                "ALTER TABLE experiences ADD COLUMN execution_multiplier REAL"
            )
        for column, declaration in (("scene_heading", "REAL"), ("scene_frame", "TEXT")):
            if column not in columns:
                self._db.execute(
                    f"ALTER TABLE experiences ADD COLUMN {column} {declaration}"
                )
        self._db.execute(
            "CREATE INDEX IF NOT EXISTS experiences_lookup "
            "ON experiences(encoder_id, map_id, embedding_dim, heading_count, id DESC)"
        )
        self._db.commit()

    @staticmethod
    def _unit_vector(values: np.ndarray) -> Optional[np.ndarray]:
        vector = np.asarray(values, dtype=np.float32).reshape(-1)
        if vector.size == 0 or not np.all(np.isfinite(vector)):
            return None
        norm = float(np.linalg.norm(vector))
        return None if norm <= 1e-8 else np.ascontiguousarray(vector / norm)

    def retrieve(
        self, embedding: np.ndarray, heading_count: int
    ) -> Tuple[Optional[np.ndarray], float]:
        """Compatibility [0,1] penalty prior; production uses execution evidence."""
        return self._retrieve(embedding, heading_count, execution=False)

    def retrieve_execution_prior(
        self,
        embedding: np.ndarray,
        heading_count: int,
        *,
        current_heading=None,
        heading_frame=None,
    ) -> Tuple[Optional[np.ndarray], float]:
        """Return [0,2] multipliers: unknown=1, measured success>1, failure<1."""
        return self._retrieve(
            embedding,
            heading_count,
            execution=True,
            current_heading=current_heading,
            heading_frame=heading_frame,
        )

    def _retrieve(
        self,
        embedding,
        heading_count,
        *,
        execution,
        current_heading=None,
        heading_frame=None,
    ):
        query = self._unit_vector(embedding)
        heading_count = int(heading_count)
        if query is None or heading_count <= 0:
            return None, 0.0
        rows = self._db.execute(
            """SELECT embedding, heading_scores, supervised_mask, execution_multiplier, scene_heading, scene_frame FROM experiences
               WHERE encoder_id=? AND map_id=? AND embedding_dim=? AND heading_count=?
               ORDER BY id DESC LIMIT ?""",
            (
                self.encoder_id,
                self.map_id,
                query.size,
                heading_count,
                self.retrieval_candidates,
            ),
        ).fetchall()
        matches = []
        for (
            embedding_blob,
            scores_blob,
            mask_blob,
            multiplier,
            scene_heading,
            scene_frame,
        ) in rows:
            stored = np.frombuffer(embedding_blob, dtype=np.float32)
            scores = np.frombuffer(scores_blob, dtype=np.float32)
            if stored.size != query.size or scores.size != heading_count:
                continue
            similarity = float(np.clip(np.dot(query, stored), -1.0, 1.0))
            # NULL preserves legacy records. New attempts supervise only their
            # executed bin; unsupported directions must remain neutral.
            mask = (
                np.ones(heading_count, dtype=bool)
                if mask_blob is None
                else np.frombuffer(mask_blob, dtype=np.uint8).astype(bool)
            )
            if mask.size != heading_count or not mask.any():
                continue
            if execution and multiplier is not None:
                if (
                    not np.isfinite(multiplier)
                    or not 0.0 <= multiplier <= 2.0
                    or scene_heading is None
                    or not np.isfinite(scene_heading)
                    or current_heading is None
                    or not np.isfinite(current_heading)
                    or not scene_frame
                    or scene_frame != heading_frame
                ):
                    continue
                # Descriptors describe the view relative to the carrier. Align
                # its executed bearing to the current view before scoring bins.
                shift = int(
                    np.round(
                        (current_heading - scene_heading)
                        * heading_count
                        / (2.0 * np.pi)
                    )
                )
                mask = np.roll(mask, shift % heading_count)
                scores = np.where(mask, multiplier, 1.0).astype(np.float32)
            if similarity >= self.similarity_threshold and np.all(np.isfinite(scores)):
                matches.append((similarity, scores.copy(), mask))
        if not matches:
            return None, 0.0
        matches.sort(key=lambda row: row[0], reverse=True)
        matches = matches[:5]
        weights = np.asarray(
            [max(row[0], 1e-3) ** 4 for row in matches], dtype=np.float32
        )
        values = np.stack([row[1] for row in matches])
        support = np.stack([row[2] for row in matches]) * weights[:, None]
        total = support.sum(axis=0)
        prior = np.ones(heading_count, dtype=np.float32)
        np.divide((values * support).sum(axis=0), total, out=prior, where=total > 0)
        return np.clip(prior, 0.0, 2.0 if execution else 1.0).astype(
            np.float32
        ), matches[0][0]

    def record_feedback(
        self,
        embedding: np.ndarray,
        heading_scores: np.ndarray,
        commanded_heading: float,
        reward: float,
        outcome: int,
        *,
        heading_count: int = 16,
        scene_heading=None,
        scene_frame=None,
    ) -> bool:
        """Store one attempt, moving its chosen heading toward observed reward."""
        vector = self._unit_vector(embedding)
        if heading_scores is None:
            if not isinstance(heading_count, int) or not 1 <= heading_count <= 1024:
                return False
            scores = np.ones(heading_count, dtype=np.float32)
        else:
            scores = np.asarray(heading_scores, dtype=np.float32).reshape(-1).copy()
        if vector is None or scores.size == 0 or not np.all(np.isfinite(scores)):
            return False
        scores = np.clip(scores, 0.0, 1.0)
        if not np.isfinite(commanded_heading) or not np.isfinite(reward):
            return False
        if scene_heading is not None and (
            not np.isfinite(scene_heading) or not scene_frame
        ):
            return False
        reward = float(np.clip(reward, 0.0, 1.0))
        bin_size = (2.0 * np.pi) / scores.size
        idx = (
            int(np.round((float(commanded_heading) % (2.0 * np.pi)) / bin_size))
            % scores.size
        )
        scores[idx] = (1.0 - self.label_strength) * scores[
            idx
        ] + self.label_strength * reward
        mask = np.zeros(scores.size, dtype=np.uint8)
        mask[idx] = 1
        self._db.execute(
            """INSERT INTO experiences
               (encoder_id, map_id, embedding, embedding_dim, heading_scores, heading_count, outcome, created_at, supervised_mask, execution_multiplier, scene_heading, scene_frame)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
            (
                self.encoder_id,
                self.map_id,
                vector.tobytes(),
                vector.size,
                np.ascontiguousarray(scores).tobytes(),
                scores.size,
                int(outcome),
                time.time(),
                mask.tobytes(),
                None
                if scene_heading is None
                else 1.0 + self.label_strength * (2.0 * reward - 1.0),
                scene_heading,
                scene_frame,
            ),
        )
        self._db.execute(
            """DELETE FROM experiences WHERE encoder_id=? AND map_id=? AND id NOT IN
                          (SELECT id FROM experiences WHERE encoder_id=? AND map_id=?
                           ORDER BY id DESC LIMIT ?)""",
            (
                self.encoder_id,
                self.map_id,
                self.encoder_id,
                self.map_id,
                self.max_records,
            ),
        )
        self._db.commit()
        return True

    def close(self) -> None:
        self._db.close()
