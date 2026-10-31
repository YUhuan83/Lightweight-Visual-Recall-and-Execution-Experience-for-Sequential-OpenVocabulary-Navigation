import sqlite3
from contextlib import closing
from pathlib import Path
import tempfile
import unittest

import numpy as np

from visual_navigation.wildos.execution_experience_store import ExecutionExperienceStore


class VisualMemoryTests(unittest.TestCase):
    def test_in_process_memory_is_forgotten_after_restart(self):
        embedding = np.array([1.0, 0.0], dtype=np.float32)
        with self._memory(":memory:", "east") as memory:
            self.assertTrue(memory.record_feedback(
                embedding, [0.2, 0.8], 0.0, 1.0, 1))
            self.assertIsNotNone(memory.retrieve(embedding, 2)[0])
        with self._memory(":memory:", "east") as restarted:
            self.assertEqual(restarted.retrieve(embedding, 2), (None, 0.0))

    def test_map_isolation_and_per_map_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "memory.sqlite3")
            emb = np.array([1.0, 0.0], dtype=np.float32)
            with self._memory(path, "east", max_records=1) as east:
                self.assertTrue(east.record_feedback(emb, [0.2, 0.8], 0.0, 1.0, 1))
                self.assertTrue(east.record_feedback(emb, [0.3, 0.7], 0.0, 0.0, 2))
            with self._memory(path, "ring", max_records=1) as ring:
                self.assertEqual(ring.retrieve(emb, 2), (None, 0.0))
                self.assertTrue(ring.record_feedback(emb, [0.7, 0.3], 0.0, 1.0, 1))
            with self._memory(path, "east", max_records=1) as east:
                prior, similarity = east.retrieve(emb, 2)
                self.assertAlmostEqual(similarity, 1.0)
                self.assertLess(prior[0], 0.1)
            with closing(sqlite3.connect(path)) as db:
                self.assertEqual(dict(db.execute(
                    "SELECT map_id, COUNT(*) FROM experiences GROUP BY map_id")),
                    {"east": 1, "ring": 1})

    def test_legacy_database_migrates_without_losing_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "legacy.sqlite3")
            with closing(sqlite3.connect(path)) as db:
                db.execute("""CREATE TABLE experiences (
                    id INTEGER PRIMARY KEY AUTOINCREMENT, encoder_id TEXT NOT NULL,
                    embedding BLOB NOT NULL, embedding_dim INTEGER NOT NULL,
                    heading_scores BLOB NOT NULL, heading_count INTEGER NOT NULL,
                    outcome INTEGER NOT NULL, created_at REAL NOT NULL)""")
                db.execute("INSERT INTO experiences (encoder_id,embedding,embedding_dim,heading_scores,heading_count,outcome,created_at) VALUES (?,?,?,?,?,?,?)",
                           ("radio", np.array([1, 0], np.float32).tobytes(), 2,
                            np.array([0.4, 0.6], np.float32).tobytes(), 2, 1, 1.0))
                db.commit()
            with self._memory(path, "default") as memory:
                prior, _ = memory.retrieve(np.array([1, 0], np.float32), 2)
                np.testing.assert_allclose(prior, [0.4, 0.6])

    def test_nonfinite_feedback_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with self._memory(str(Path(directory) / "memory.sqlite3"), "east") as memory:
                self.assertFalse(memory.record_feedback([1, 0], [0.5, 0.5], float("nan"), 1, 1))
                self.assertFalse(memory.record_feedback([1, 0], [0.5, 0.5], 0, float("nan"), 1))

    @staticmethod
    def _memory(path, map_id, max_records=4096):
        return _MemoryContext(ExecutionExperienceStore(path, "radio", map_id=map_id, max_records=max_records))


class _MemoryContext:
    def __init__(self, memory):
        self.memory = memory

    def __enter__(self):
        return self.memory

    def __exit__(self, *_):
        self.memory.close()
