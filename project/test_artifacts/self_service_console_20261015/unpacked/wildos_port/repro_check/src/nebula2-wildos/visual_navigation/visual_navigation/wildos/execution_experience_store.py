"""Execution feedback history for scene-conditioned heading priors.

SQLite ':memory:' keeps the store within this process. Storage lifetime is
chosen by the existing configuration; this class does not change that policy.
"""

import sqlite3
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np


class ExecutionExperienceStore:
    """CPU-friendly SQLite index of RADIO descriptors and heading targets."""

    def __init__(self, database_path: str, encoder_id: str, max_records: int = 4096,
                 similarity_threshold: float = 0.82, label_strength: float = 0.8,
                 map_id: str = "default"):
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
            self._db.execute("ALTER TABLE experiences ADD COLUMN map_id TEXT NOT NULL DEFAULT 'default'")
        self._db.execute("CREATE INDEX IF NOT EXISTS experiences_lookup "
                          "ON experiences(encoder_id, map_id, embedding_dim, heading_count, id DESC)")
        self._db.commit()

    @staticmethod
    def _unit_vector(values: np.ndarray) -> Optional[np.ndarray]:
        vector = np.asarray(values, dtype=np.float32).reshape(-1)
        if vector.size == 0 or not np.all(np.isfinite(vector)):
            return None
        norm = float(np.linalg.norm(vector))
        return None if norm <= 1e-8 else np.ascontiguousarray(vector / norm)

    def retrieve(self, embedding: np.ndarray, heading_count: int) -> Tuple[Optional[np.ndarray], float]:
        """Return a similarity-weighted prior, or None when no scene matches."""
        query = self._unit_vector(embedding)
        heading_count = int(heading_count)
        if query is None or heading_count <= 0:
            return None, 0.0
        rows = self._db.execute(
            """SELECT embedding, heading_scores FROM experiences
               WHERE encoder_id=? AND map_id=? AND embedding_dim=? AND heading_count=?
               ORDER BY id DESC LIMIT ?""",
            (self.encoder_id, self.map_id, query.size, heading_count, self.retrieval_candidates),
        ).fetchall()
        matches = []
        for embedding_blob, scores_blob in rows:
            stored = np.frombuffer(embedding_blob, dtype=np.float32)
            scores = np.frombuffer(scores_blob, dtype=np.float32)
            if stored.size != query.size or scores.size != heading_count:
                continue
            similarity = float(np.clip(np.dot(query, stored), -1.0, 1.0))
            if similarity >= self.similarity_threshold and np.all(np.isfinite(scores)):
                matches.append((similarity, scores.copy()))
        if not matches:
            return None, 0.0
        matches.sort(key=lambda row: row[0], reverse=True)
        matches = matches[:5]
        weights = np.asarray([max(sim, 1e-3) ** 4 for sim, _ in matches], dtype=np.float32)
        prior = np.average(np.stack([scores for _, scores in matches]), axis=0, weights=weights)
        return np.clip(prior, 0.0, 1.0).astype(np.float32), matches[0][0]

    def record_feedback(self, embedding: np.ndarray, heading_scores: np.ndarray,
                        commanded_heading: float, reward: float, outcome: int) -> bool:
        """Store one attempt, moving its chosen heading toward observed reward."""
        vector = self._unit_vector(embedding)
        scores = np.asarray(heading_scores, dtype=np.float32).reshape(-1).copy()
        if vector is None or scores.size == 0 or not np.all(np.isfinite(scores)):
            return False
        scores = np.clip(scores, 0.0, 1.0)
        if not np.isfinite(commanded_heading) or not np.isfinite(reward):
            return False
        reward = float(np.clip(reward, 0.0, 1.0))
        bin_size = (2.0 * np.pi) / scores.size
        idx = int(np.round((float(commanded_heading) % (2.0 * np.pi)) / bin_size)) % scores.size
        scores[idx] = (1.0 - self.label_strength) * scores[idx] + self.label_strength * reward
        self._db.execute(
            """INSERT INTO experiences
               (encoder_id, map_id, embedding, embedding_dim, heading_scores, heading_count, outcome, created_at)
               VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
            (self.encoder_id, self.map_id, vector.tobytes(), vector.size, np.ascontiguousarray(scores).tobytes(),
             scores.size, int(outcome), time.time()),
        )
        self._db.execute("""DELETE FROM experiences WHERE encoder_id=? AND map_id=? AND id NOT IN
                          (SELECT id FROM experiences WHERE encoder_id=? AND map_id=?
                           ORDER BY id DESC LIMIT ?)""",
                          (self.encoder_id, self.map_id, self.encoder_id, self.map_id, self.max_records))
        self._db.commit()
        return True

    def close(self) -> None:
        self._db.close()
