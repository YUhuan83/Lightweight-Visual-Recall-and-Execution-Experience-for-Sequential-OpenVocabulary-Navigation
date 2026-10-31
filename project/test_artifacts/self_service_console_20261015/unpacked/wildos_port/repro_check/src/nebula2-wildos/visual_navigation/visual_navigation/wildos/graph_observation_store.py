"""Visual observations and objects anchored to the existing navigation graph.

This store does not build a second geometric map. Existing database schema and
configuration remain compatible, including single-process ':memory:' mode.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sqlite3
import time
from typing import Iterable, Optional, Sequence
import uuid

import numpy as np

from visual_navigation.wildos.relation_query import SemanticObject, resolve_relation


@dataclass(frozen=True)
class NodeObservationMatch:
    map_id: str
    node_uuid: str
    heading_bin: int
    heading: float
    position: tuple[float, float, float]
    similarity: float
    heading_compatibility: float
    position_compatibility: float
    score: float
    observation_count: int
    image_path: Optional[str]


# Backward-compatible type alias.  It no longer represents an independent
# SLAM keyframe: every record is anchored to an existing graph node and bin.
KeyframeMatch = NodeObservationMatch


@dataclass(frozen=True)
class MapMatch:
    map_id: str
    node_uuid: str
    heading_bin: int
    similarity: float
    heading_compatibility: float
    score: float


@dataclass(frozen=True)
class RememberedObject:
    object_id: str
    label: str
    position: tuple[float, float, float]
    confidence: float
    association_confidence: float
    observation_count: int
    best_view_pose: Optional[tuple[float, float, float, float]]
    approach_pose: Optional[tuple[float, float, float, float]]
    anchor_node_uuid: Optional[str]
    anchor_heading_bin: Optional[int]
    best_view_node_uuid: Optional[str]
    best_view_heading_bin: Optional[int]
    approach_node_uuid: Optional[str]
    approach_heading_bin: Optional[int]
    reached: bool
    blocked: bool
    dynamic_probability: float
    status: str
    independent_observation_count: int = 0
    unique_node_count: int = 0
    prototype_count: int = 0


class GraphObservationStore:
    """SQLite observation store, independent from ROS message types."""

    SCHEMA_VERSION = 3

    def __init__(self, database_path: str, encoder_id: str,
                 visual_similarity_threshold: float = 0.82,
                 object_similarity_threshold: float = 0.82,
                 object_max_distance: float = 3.0,
                 object_confirmation_observations: int = 2,
                 object_independent_heading_bins: int = 2,
                 object_heading_bin_count: int = 16,
                 object_prototype_novelty_threshold: float = 0.95):
        self.path = Path(database_path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.encoder_id = str(encoder_id)
        self.visual_similarity_threshold = float(visual_similarity_threshold)
        self.object_similarity_threshold = float(object_similarity_threshold)
        self.object_max_distance = float(object_max_distance)
        self.object_confirmation_observations = max(1, int(object_confirmation_observations))
        self.object_independent_heading_bins = max(1, int(object_independent_heading_bins))
        self.object_heading_bin_count = max(1, int(object_heading_bin_count))
        self.object_prototype_novelty_threshold = float(object_prototype_novelty_threshold)
        self._db = sqlite3.connect(str(self.path), timeout=10.0)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.execute("PRAGMA foreign_keys=ON")
        self._create_schema()

    def _create_schema(self):
        self._db.executescript("""
        CREATE TABLE IF NOT EXISTS maps (
          map_id TEXT PRIMARY KEY, map_name TEXT NOT NULL, encoder_id TEXT NOT NULL,
          schema_version INTEGER NOT NULL, geometry_hash TEXT,
          created_at REAL NOT NULL, last_used REAL NOT NULL
        );
        CREATE TABLE IF NOT EXISTS keyframes (
          keyframe_id INTEGER PRIMARY KEY AUTOINCREMENT, map_id TEXT NOT NULL,
          node_uuid TEXT, x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
          yaw REAL NOT NULL, embedding BLOB NOT NULL, embedding_dim INTEGER NOT NULL,
          image_path TEXT, traversability_mean REAL, traversability_std REAL,
          frontier_count INTEGER, observed_at REAL NOT NULL,
          FOREIGN KEY(map_id) REFERENCES maps(map_id) ON DELETE CASCADE
        );
        CREATE INDEX IF NOT EXISTS keyframes_map ON keyframes(map_id, observed_at);
        CREATE TABLE IF NOT EXISTS node_observations (
          map_id TEXT NOT NULL, node_uuid TEXT NOT NULL, heading_bin INTEGER NOT NULL,
          heading REAL NOT NULL, x REAL, y REAL, z REAL,
          embedding BLOB NOT NULL, embedding_dim INTEGER NOT NULL,
          image_path TEXT, frontier_score_mean REAL, frontier_score_m2 REAL,
          semantic_traversability_mean REAL, physical_traversability_mean REAL,
          dynamic_risk_mean REAL, uncertainty_mean REAL, predicted_utility_mean REAL,
          first_seen REAL NOT NULL, last_seen REAL NOT NULL,
          observation_count INTEGER NOT NULL,
          PRIMARY KEY(map_id,node_uuid,heading_bin),
          FOREIGN KEY(map_id) REFERENCES maps(map_id) ON DELETE CASCADE
        );
        CREATE INDEX IF NOT EXISTS node_observations_embedding
          ON node_observations(map_id,embedding_dim,last_seen);
        CREATE TABLE IF NOT EXISTS semantic_objects (
          object_id TEXT PRIMARY KEY, map_id TEXT NOT NULL, label TEXT NOT NULL,
          x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
          covariance BLOB, embedding BLOB, embedding_dim INTEGER,
           confidence REAL NOT NULL, association_confidence REAL NOT NULL,
           first_seen REAL NOT NULL, last_seen REAL NOT NULL,
           observation_count INTEGER NOT NULL,
           independent_observation_count INTEGER NOT NULL DEFAULT 1,
           unique_node_count INTEGER NOT NULL DEFAULT 0,
           embedding_weight REAL NOT NULL DEFAULT 1.0,
           prototype_2 BLOB, prototype_2_dim INTEGER,
           prototype_2_weight REAL NOT NULL DEFAULT 0.0,
           best_view_x REAL, best_view_y REAL, best_view_z REAL, best_view_yaw REAL,
          approach_x REAL, approach_y REAL, approach_z REAL, approach_yaw REAL,
          anchor_node_uuid TEXT, anchor_heading_bin INTEGER,
          best_view_node_uuid TEXT, best_view_heading_bin INTEGER,
          approach_node_uuid TEXT, approach_heading_bin INTEGER,
          reached INTEGER NOT NULL DEFAULT 0, blocked INTEGER NOT NULL DEFAULT 0,
           dynamic_probability REAL NOT NULL DEFAULT 0.0,
           status TEXT NOT NULL DEFAULT 'TENTATIVE',
          FOREIGN KEY(map_id) REFERENCES maps(map_id) ON DELETE CASCADE
        );
        CREATE INDEX IF NOT EXISTS objects_map_label ON semantic_objects(map_id, label);
        CREATE TABLE IF NOT EXISTS object_observations (
          observation_id INTEGER PRIMARY KEY AUTOINCREMENT,
          object_id TEXT NOT NULL, keyframe_id INTEGER,
           node_uuid TEXT, heading_bin INTEGER,
           independent INTEGER NOT NULL DEFAULT 1,
           confidence REAL NOT NULL, viewing_heading REAL,
          distance REAL, image_path TEXT, observed_at REAL NOT NULL,
          FOREIGN KEY(object_id) REFERENCES semantic_objects(object_id) ON DELETE CASCADE,
          FOREIGN KEY(keyframe_id) REFERENCES keyframes(keyframe_id) ON DELETE SET NULL
        );
        CREATE TABLE IF NOT EXISTS search_coverage (
          map_id TEXT NOT NULL, query TEXT NOT NULL, node_uuid TEXT NOT NULL,
          heading_mask INTEGER NOT NULL, negative_evidence REAL NOT NULL,
          observation_count INTEGER NOT NULL,
          first_searched REAL NOT NULL, last_searched REAL NOT NULL,
          PRIMARY KEY(map_id, query, node_uuid),
          FOREIGN KEY(map_id) REFERENCES maps(map_id) ON DELETE CASCADE
        );
        CREATE TABLE IF NOT EXISTS traversal_history (
          id INTEGER PRIMARY KEY AUTOINCREMENT, map_id TEXT NOT NULL,
          from_node TEXT, to_node TEXT, robot_profile_id TEXT,
          start_heading_bin INTEGER, end_heading_bin INTEGER,
          outcome INTEGER NOT NULL, duration REAL,
          commanded_distance REAL, observed_distance REAL,
          slip_ratio REAL, energy_cost REAL, observed_at REAL NOT NULL,
          FOREIGN KEY(map_id) REFERENCES maps(map_id) ON DELETE CASCADE
        );
        CREATE INDEX IF NOT EXISTS traversal_edge ON traversal_history(map_id, from_node, to_node);
        """)
        # Migrate existing v1 databases without deleting persisted memories.
        added_columns = set()
        for table, column, declaration in (
            ("semantic_objects", "anchor_node_uuid", "TEXT"),
            ("semantic_objects", "anchor_heading_bin", "INTEGER"),
            ("semantic_objects", "best_view_node_uuid", "TEXT"),
            ("semantic_objects", "best_view_heading_bin", "INTEGER"),
            ("semantic_objects", "approach_node_uuid", "TEXT"),
            ("semantic_objects", "approach_heading_bin", "INTEGER"),
            ("semantic_objects", "independent_observation_count", "INTEGER NOT NULL DEFAULT 1"),
            ("semantic_objects", "unique_node_count", "INTEGER NOT NULL DEFAULT 0"),
            ("semantic_objects", "embedding_weight", "REAL NOT NULL DEFAULT 1.0"),
            ("semantic_objects", "prototype_2", "BLOB"),
            ("semantic_objects", "prototype_2_dim", "INTEGER"),
            ("semantic_objects", "prototype_2_weight", "REAL NOT NULL DEFAULT 0.0"),
            ("object_observations", "node_uuid", "TEXT"),
            ("object_observations", "heading_bin", "INTEGER"),
            ("object_observations", "independent", "INTEGER NOT NULL DEFAULT 1"),
        ):
            if self._ensure_column(table, column, declaration):
                added_columns.add((table, column))
        self._db.execute("""CREATE INDEX IF NOT EXISTS object_observations_identity
                            ON object_observations(object_id,node_uuid)""")
        legacy_active = self._db.execute(
            "SELECT 1 FROM semantic_objects WHERE status='ACTIVE' LIMIT 1").fetchone()
        if ("semantic_objects", "unique_node_count") in added_columns or legacy_active:
            self._db.execute("""
              UPDATE semantic_objects SET unique_node_count=(
                SELECT COUNT(DISTINCT node_uuid) FROM object_observations
                WHERE object_observations.object_id=semantic_objects.object_id
                  AND node_uuid IS NOT NULL)
            """)
            self._db.execute("""
              UPDATE semantic_objects SET
                independent_observation_count=MAX(1, unique_node_count),
                status=CASE WHEN unique_node_count>=? THEN 'CONFIRMED' ELSE 'TENTATIVE' END
              WHERE status='ACTIVE'
            """, (self.object_confirmation_observations,))
        self._db.commit()

    def _ensure_column(self, table: str, column: str, declaration: str):
        columns = {row[1] for row in self._db.execute(f"PRAGMA table_info({table})")}
        if column not in columns:
            self._db.execute(f"ALTER TABLE {table} ADD COLUMN {column} {declaration}")
            return True
        return False

    @staticmethod
    def _vector(values) -> Optional[np.ndarray]:
        if values is None:
            return None
        vector = np.asarray(values, dtype=np.float32).reshape(-1)
        if vector.size == 0 or not np.all(np.isfinite(vector)):
            return None
        return np.ascontiguousarray(vector)

    @classmethod
    def _unit(cls, values) -> Optional[np.ndarray]:
        vector = cls._vector(values)
        if vector is None:
            return None
        norm = float(np.linalg.norm(vector))
        return None if norm <= 1e-8 else np.ascontiguousarray(vector / norm)

    @staticmethod
    def _angle_difference(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    @staticmethod
    def _pose_columns(pose: Optional[Sequence[float]]):
        if pose is None:
            return (None, None, None, None)
        values = tuple(float(v) for v in pose)
        if len(values) != 4:
            raise ValueError("pose must be (x, y, z, yaw)")
        return values

    def register_map(self, map_id: str, map_name: Optional[str] = None,
                     geometry_hash: Optional[str] = None, timestamp: Optional[float] = None):
        now = float(time.time() if timestamp is None else timestamp)
        self._db.execute("""
          INSERT INTO maps(map_id,map_name,encoder_id,schema_version,geometry_hash,created_at,last_used)
          VALUES(?,?,?,?,?,?,?)
          ON CONFLICT(map_id) DO UPDATE SET map_name=excluded.map_name,
            encoder_id=excluded.encoder_id, schema_version=excluded.schema_version,
            geometry_hash=COALESCE(excluded.geometry_hash,maps.geometry_hash), last_used=excluded.last_used
        """, (map_id, map_name or map_id, self.encoder_id, self.SCHEMA_VERSION,
              geometry_hash, now, now))
        self._db.commit()

    def clear_map(self, map_id: str):
        self._db.execute("DELETE FROM maps WHERE map_id=?", (map_id,))
        self._db.commit()

    @staticmethod
    def heading_to_bin(heading: float, bin_count: int = 16) -> int:
        size = 2.0 * math.pi / max(int(bin_count), 1)
        return int(round((float(heading) % (2.0 * math.pi)) / size)) % max(int(bin_count), 1)

    @classmethod
    def should_update_node_observation(cls, last_node_uuid, last_heading_bin,
                                       last_embedding, node_uuid, heading_bin,
                                       embedding, min_embedding_distance=0.18) -> bool:
        if not last_node_uuid or last_embedding is None:
            return True
        if str(last_node_uuid) != str(node_uuid) or int(last_heading_bin) != int(heading_bin):
            return True
        a, b = cls._unit(last_embedding), cls._unit(embedding)
        return a is None or b is None or (1.0 - float(np.dot(a, b))) >= min_embedding_distance

    def observe_node_heading(self, map_id: str, node_uuid: str, heading_bin: int,
                             heading: float, position: Sequence[float], embedding,
                             image_path: Optional[str] = None,
                             frontier_score: Optional[float] = None,
                             semantic_traversability: Optional[float] = None,
                             physical_traversability: Optional[float] = None,
                             dynamic_risk: Optional[float] = None,
                             uncertainty: Optional[float] = None,
                             predicted_utility: Optional[float] = None,
                             timestamp: Optional[float] = None):
        """Fuse one visual observation into an existing graph node and heading bin."""
        vector = self._unit(embedding)
        if vector is None:
            raise ValueError("invalid node observation embedding")
        now = float(time.time() if timestamp is None else timestamp)
        x, y, z = (float(v) for v in position[:3])
        row = self._db.execute("""
          SELECT * FROM node_observations
          WHERE map_id=? AND node_uuid=? AND heading_bin=?
        """, (map_id, node_uuid, int(heading_bin))).fetchone()
        if row is None:
            self._db.execute("""
              INSERT INTO node_observations(map_id,node_uuid,heading_bin,heading,x,y,z,
                embedding,embedding_dim,image_path,frontier_score_mean,frontier_score_m2,
                semantic_traversability_mean,physical_traversability_mean,dynamic_risk_mean,
                uncertainty_mean,predicted_utility_mean,first_seen,last_seen,observation_count)
              VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """, (map_id, node_uuid, int(heading_bin), float(heading), x, y, z,
                  vector.tobytes(), vector.size, image_path,
                  frontier_score, 0.0 if frontier_score is not None else None,
                  semantic_traversability, physical_traversability, dynamic_risk,
                  uncertainty, predicted_utility, now, now, 1))
        else:
            count = int(row["observation_count"])
            old_vector = np.frombuffer(row["embedding"], dtype=np.float32)
            fused = self._unit(old_vector * count + vector)
            def mean(old, new):
                if new is None:
                    return old
                return float(new) if old is None else (float(old) * count + float(new)) / (count + 1)
            old_frontier = row["frontier_score_mean"]
            frontier_mean = mean(old_frontier, frontier_score)
            frontier_m2 = row["frontier_score_m2"]
            if frontier_score is not None:
                frontier_m2 = float(frontier_m2 or 0.0)
                if old_frontier is not None:
                    frontier_m2 += (float(frontier_score) - float(old_frontier)) * (
                        float(frontier_score) - float(frontier_mean))
            self._db.execute("""
              UPDATE node_observations SET heading=?,x=?,y=?,z=?,embedding=?,embedding_dim=?,
                image_path=COALESCE(?,image_path),frontier_score_mean=?,frontier_score_m2=?,
                semantic_traversability_mean=?,physical_traversability_mean=?,dynamic_risk_mean=?,
                uncertainty_mean=?,predicted_utility_mean=?,last_seen=?,observation_count=?
              WHERE map_id=? AND node_uuid=? AND heading_bin=?
            """, (float(heading), x, y, z, fused.tobytes(), fused.size, image_path,
                  frontier_mean, frontier_m2,
                  mean(row["semantic_traversability_mean"], semantic_traversability),
                  mean(row["physical_traversability_mean"], physical_traversability),
                  mean(row["dynamic_risk_mean"], dynamic_risk),
                  mean(row["uncertainty_mean"], uncertainty),
                  mean(row["predicted_utility_mean"], predicted_utility),
                  now, count + 1, map_id, node_uuid, int(heading_bin)))
        self._db.execute("UPDATE maps SET last_used=? WHERE map_id=?", (now, map_id))
        self._db.commit()
        return (str(node_uuid), int(heading_bin))

    def query_node_observations(self, map_id: str, embedding, yaw: Optional[float] = None,
                                position: Optional[Sequence[float]] = None,
                                node_uuid: Optional[str] = None,
                                top_k: int = 5,
                                heading_sigma: float = math.radians(45),
                                position_sigma: float = 15.0) -> list[NodeObservationMatch]:
        query = self._unit(embedding)
        if query is None:
            return []
        sql = "SELECT * FROM node_observations WHERE map_id=? AND embedding_dim=?"
        args = [map_id, query.size]
        if node_uuid is not None:
            sql += " AND node_uuid=?"
            args.append(node_uuid)
        sql += " ORDER BY last_seen DESC"
        rows = self._db.execute(sql, args).fetchall()
        matches = []
        for row in rows:
            stored = np.frombuffer(row["embedding"], dtype=np.float32)
            similarity = float(np.clip(np.dot(query, stored), -1.0, 1.0))
            if similarity < self.visual_similarity_threshold:
                continue
            heading = 1.0
            if yaw is not None:
                delta = self._angle_difference(float(yaw), row["heading"])
                heading = math.exp(-(delta * delta) / (2.0 * heading_sigma * heading_sigma))
            position_score = 1.0
            if position is not None and row["x"] is not None:
                distance = float(np.linalg.norm(
                    np.asarray(position, dtype=float)[:3] - np.array([row["x"], row["y"], row["z"]])))
                position_score = math.exp(-(distance * distance) / (2.0 * position_sigma * position_sigma))
            score = similarity * heading * position_score
            matches.append(NodeObservationMatch(
                map_id, row["node_uuid"], int(row["heading_bin"]), float(row["heading"]),
                (row["x"], row["y"], row["z"]), similarity, heading,
                position_score, score, int(row["observation_count"]), row["image_path"]))
        matches.sort(key=lambda value: value.score, reverse=True)
        return matches[:max(1, int(top_k))]

    def associate_historical_node(self, map_id: str, embedding,
                                  position: Optional[Sequence[float]] = None,
                                  yaw: Optional[float] = None,
                                  top_k: int = 5) -> list[NodeObservationMatch]:
        return self.query_node_observations(
            map_id, embedding, yaw=yaw, position=position, top_k=top_k)

    def get_node_observation(self, map_id: str, node_uuid: str, heading_bin: int):
        row = self._db.execute("""
          SELECT * FROM node_observations
          WHERE map_id=? AND node_uuid=? AND heading_bin=?
        """, (map_id, node_uuid, int(heading_bin))).fetchone()
        if row is None:
            return None
        result = dict(row)
        result["embedding"] = np.frombuffer(row["embedding"], dtype=np.float32).copy()
        result["frontier_score_std"] = math.sqrt(
            max(float(row["frontier_score_m2"] or 0.0) /
                max(int(row["observation_count"]) - 1, 1), 0.0))
        return result

    # Compatibility wrappers: no independent spatial keyframe is created.
    def add_keyframe(self, map_id: str, node_uuid: str, position: Sequence[float],
                     yaw: float, embedding, image_path: Optional[str] = None,
                     traversability_mean: Optional[float] = None,
                     traversability_std: Optional[float] = None,
                     frontier_count: Optional[int] = None,
                     timestamp: Optional[float] = None):
        heading_bin = self.heading_to_bin(yaw)
        return self.observe_node_heading(
            map_id, node_uuid, heading_bin, yaw, position, embedding,
            image_path=image_path, semantic_traversability=traversability_mean,
            uncertainty=traversability_std, timestamp=timestamp)

    def query_keyframes(self, map_id: str, embedding, yaw: Optional[float] = None,
                        top_k: int = 5, heading_sigma: float = math.radians(45)):
        return self.query_node_observations(
            map_id, embedding, yaw=yaw, top_k=top_k, heading_sigma=heading_sigma)

    def identify_maps(self, embedding, yaw: Optional[float] = None,
                      top_k: int = 3) -> list[MapMatch]:
        """Identify the most likely persisted maps using their best keyframe."""
        query = self._unit(embedding)
        if query is None:
            return []
        map_ids = [row[0] for row in self._db.execute(
            "SELECT map_id FROM maps WHERE encoder_id=?", (self.encoder_id,)).fetchall()]
        matches = []
        for map_id in map_ids:
            result = self.query_keyframes(map_id, query, yaw=yaw, top_k=1)
            if result:
                match = result[0]
                matches.append(MapMatch(
                    map_id, match.node_uuid, match.heading_bin, match.similarity,
                    match.heading_compatibility, match.score))
        matches.sort(key=lambda value: value.score, reverse=True)
        return matches[:max(1, int(top_k))]

    def observe_object(self, map_id: str, label: str, position: Sequence[float],
                       embedding=None, confidence: float = 1.0,
                       covariance=None, keyframe_id: Optional[int] = None,
                       anchor_node_uuid: Optional[str] = None,
                       anchor_heading_bin: Optional[int] = None,
                       observer_pose: Optional[Sequence[float]] = None,
                       best_view_node_uuid: Optional[str] = None,
                       best_view_heading_bin: Optional[int] = None,
                       approach_pose: Optional[Sequence[float]] = None,
                       approach_node_uuid: Optional[str] = None,
                       approach_heading_bin: Optional[int] = None,
                       image_path: Optional[str] = None,
                       dynamic_probability: float = 0.0,
                       timestamp: Optional[float] = None) -> str:
        now = float(time.time() if timestamp is None else timestamp)
        position = np.asarray(position, dtype=np.float64)[:3]
        vector = self._unit(embedding)
        rows = self._db.execute(
            "SELECT * FROM semantic_objects WHERE map_id=? AND label=? AND status!='REMOVED'",
            (map_id, label)).fetchall()
        best, best_score = None, -1.0
        for row in rows:
            distance = float(np.linalg.norm(position - np.array([row["x"], row["y"], row["z"]])))
            if distance > self.object_max_distance:
                continue
            similarity = 1.0
            if vector is not None and row["embedding"] is not None:
                prototypes = [np.frombuffer(row["embedding"], dtype=np.float32)]
                if row["prototype_2"] is not None:
                    prototypes.append(np.frombuffer(row["prototype_2"], dtype=np.float32))
                compatible = [item for item in prototypes if item.size == vector.size]
                if not compatible:
                    continue
                similarity = max(
                    float(np.clip(np.dot(vector, item), -1.0, 1.0))
                    for item in compatible)
                if similarity < self.object_similarity_threshold:
                    continue
            score = similarity * math.exp(-distance / max(self.object_max_distance, 1e-6))
            if score > best_score:
                best, best_score = row, score

        cov = self._vector(covariance)
        best_view = self._pose_columns(observer_pose)
        approach = self._pose_columns(approach_pose)
        confidence = float(np.clip(confidence, 0.0, 1.0))
        dynamic_probability = float(np.clip(dynamic_probability, 0.0, 1.0))
        if best is None:
            object_id = f"{label}_{uuid.uuid4().hex[:12]}"
            unique_node_count = int(anchor_node_uuid is not None)
            independent_count = 1
            status = ("CONFIRMED" if independent_count >= self.object_confirmation_observations
                      else "TENTATIVE")
            self._db.execute("""
              INSERT INTO semantic_objects(object_id,map_id,label,x,y,z,covariance,embedding,embedding_dim,
                confidence,association_confidence,first_seen,last_seen,observation_count,
                independent_observation_count,unique_node_count,embedding_weight,
                best_view_x,best_view_y,best_view_z,best_view_yaw,
                approach_x,approach_y,approach_z,approach_yaw,dynamic_probability,status)
              VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
            """, (object_id, map_id, label, *position,
                  cov.tobytes() if cov is not None else None,
                  vector.tobytes() if vector is not None else None,
                  vector.size if vector is not None else None,
                  confidence, max(best_score, confidence), now, now, 1,
                  independent_count, unique_node_count, max(confidence, 1e-3),
                  *best_view, *approach, dynamic_probability, status))
            self._db.execute("""
              UPDATE semantic_objects SET anchor_node_uuid=?,anchor_heading_bin=?,
                best_view_node_uuid=?,best_view_heading_bin=?,
                approach_node_uuid=?,approach_heading_bin=? WHERE object_id=?
            """, (anchor_node_uuid, anchor_heading_bin,
                  best_view_node_uuid or anchor_node_uuid,
                  best_view_heading_bin if best_view_heading_bin is not None else anchor_heading_bin,
                  approach_node_uuid, approach_heading_bin, object_id))
        else:
            object_id = best["object_id"]
            old_count = int(best["observation_count"])
            old_weight = max(float(best["confidence"]) * old_count, 1e-3)
            new_weight = max(confidence, 1e-3)
            old_position = np.array([best["x"], best["y"], best["z"]])
            fused_position = (old_position * old_weight + position * new_weight) / (old_weight + new_weight)
            fused_vector = None
            prototype_2 = best["prototype_2"]
            prototype_2_dim = best["prototype_2_dim"]
            embedding_weight = float(best["embedding_weight"] or old_weight)
            prototype_2_weight = float(best["prototype_2_weight"] or 0.0)
            if vector is not None:
                first = (np.frombuffer(best["embedding"], dtype=np.float32)
                         if best["embedding"] is not None else None)
                second = (np.frombuffer(best["prototype_2"], dtype=np.float32)
                          if best["prototype_2"] is not None else None)
                first_similarity = (-1.0 if first is None or first.size != vector.size else
                                    float(np.dot(first, vector)))
                second_similarity = (-1.0 if second is None or second.size != vector.size else
                                     float(np.dot(second, vector)))
                if first is None or first.size != vector.size:
                    fused_vector = vector
                    embedding_weight = new_weight
                elif second is None and first_similarity < self.object_prototype_novelty_threshold:
                    prototype_2 = vector.tobytes()
                    prototype_2_dim = vector.size
                    prototype_2_weight = new_weight
                elif second is not None and second_similarity > first_similarity:
                    fused_second = self._unit(second * prototype_2_weight + vector * new_weight)
                    prototype_2 = fused_second.tobytes()
                    prototype_2_dim = fused_second.size
                    prototype_2_weight += new_weight
                else:
                    fused_vector = self._unit(first * embedding_weight + vector * new_weight)
                    embedding_weight += new_weight

            previous_observations = self._db.execute("""
              SELECT node_uuid,heading_bin FROM object_observations WHERE object_id=?
            """, (object_id,)).fetchall()
            known_nodes = {row["node_uuid"] for row in previous_observations
                           if row["node_uuid"] is not None}
            independent = False
            if anchor_node_uuid is not None and anchor_node_uuid not in known_nodes:
                independent = True
            elif anchor_node_uuid is not None and anchor_heading_bin is not None:
                for observation in previous_observations:
                    if observation["node_uuid"] != anchor_node_uuid or observation["heading_bin"] is None:
                        continue
                    delta = abs(int(anchor_heading_bin) - int(observation["heading_bin"]))
                    delta %= self.object_heading_bin_count
                    delta = min(delta, self.object_heading_bin_count - delta)
                    if delta >= self.object_independent_heading_bins:
                        independent = True
                        break
            independent_count = int(best["independent_observation_count"]) + int(independent)
            unique_node_count = len(known_nodes | ({anchor_node_uuid} if anchor_node_uuid else set()))
            status = ("CONFIRMED" if independent_count >= self.object_confirmation_observations
                      else "TENTATIVE")
            update_view = confidence >= float(best["confidence"])
            values = {
                "x": fused_position[0], "y": fused_position[1], "z": fused_position[2],
                "embedding": fused_vector.tobytes() if fused_vector is not None else best["embedding"],
                "embedding_dim": fused_vector.size if fused_vector is not None else best["embedding_dim"],
                "embedding_weight": embedding_weight,
                "prototype_2": prototype_2, "prototype_2_dim": prototype_2_dim,
                "prototype_2_weight": prototype_2_weight,
                "confidence": max(float(best["confidence"]), confidence),
                "association_confidence": max(float(best["association_confidence"]), best_score),
                "last_seen": now, "observation_count": old_count + 1,
                "independent_observation_count": independent_count,
                "unique_node_count": unique_node_count, "status": status,
                "dynamic_probability": max(float(best["dynamic_probability"]), dynamic_probability),
            }
            if anchor_node_uuid is not None and best["anchor_node_uuid"] is None:
                values.update(anchor_node_uuid=anchor_node_uuid, anchor_heading_bin=anchor_heading_bin)
            if update_view and observer_pose is not None:
                values.update(dict(zip(("best_view_x", "best_view_y", "best_view_z", "best_view_yaw"), best_view)))
                view_node = best_view_node_uuid or anchor_node_uuid
                view_bin = best_view_heading_bin if best_view_heading_bin is not None else anchor_heading_bin
                if view_node is not None:
                    values["best_view_node_uuid"] = view_node
                if view_bin is not None:
                    values["best_view_heading_bin"] = view_bin
            if update_view and approach_pose is not None:
                values.update(dict(zip(("approach_x", "approach_y", "approach_z", "approach_yaw"), approach)))
                if approach_node_uuid is not None:
                    values["approach_node_uuid"] = approach_node_uuid
                if approach_heading_bin is not None:
                    values["approach_heading_bin"] = approach_heading_bin
            clause = ",".join(f"{key}=?" for key in values)
            self._db.execute(f"UPDATE semantic_objects SET {clause} WHERE object_id=?",
                             (*values.values(), object_id))

        ox, oy, oz, oyaw = best_view
        distance = None if observer_pose is None else float(np.linalg.norm(position - np.array([ox, oy, oz])))
        observation_independent = (1 if best is None else int(independent))
        self._db.execute("""
          INSERT INTO object_observations(object_id,keyframe_id,node_uuid,heading_bin,
            independent,confidence,viewing_heading,distance,image_path,observed_at)
          VALUES(?,?,?,?,?,?,?,?,?,?)
        """, (object_id, keyframe_id, anchor_node_uuid, anchor_heading_bin,
              observation_independent, confidence, oyaw, distance, image_path, now))
        self._db.commit()
        return object_id

    @staticmethod
    def _half_life(dynamic_probability: float) -> float:
        if dynamic_probability < 0.2:
            return 365.0 * 24.0 * 3600.0
        if dynamic_probability < 0.7:
            return 3600.0
        return 10.0

    def query_objects(self, map_id: str, label: Optional[str] = None,
                      min_confidence: float = 0.0, now: Optional[float] = None,
                      include_stale: bool = False,
                      confirmed_only: bool = False) -> list[RememberedObject]:
        query = "SELECT * FROM semantic_objects WHERE map_id=?"
        args = [map_id]
        if label:
            query += " AND label=?"
            args.append(label)
        if confirmed_only:
            query += " AND status='CONFIRMED'"
        rows = self._db.execute(query, args).fetchall()
        current = float(time.time() if now is None else now)
        results = []
        for row in rows:
            age = max(0.0, current - float(row["last_seen"]))
            confidence = float(row["confidence"]) * math.exp(-age / self._half_life(row["dynamic_probability"]))
            if confidence < min_confidence or (row["status"] == "STALE" and not include_stale):
                continue
            best_view = None if row["best_view_x"] is None else (
                row["best_view_x"], row["best_view_y"], row["best_view_z"], row["best_view_yaw"])
            approach = None if row["approach_x"] is None else (
                row["approach_x"], row["approach_y"], row["approach_z"], row["approach_yaw"])
            results.append(RememberedObject(
                row["object_id"], row["label"], (row["x"], row["y"], row["z"]),
                confidence, row["association_confidence"], row["observation_count"],
                best_view, approach,
                row["anchor_node_uuid"], row["anchor_heading_bin"],
                row["best_view_node_uuid"], row["best_view_heading_bin"],
                row["approach_node_uuid"], row["approach_heading_bin"],
                bool(row["reached"]), bool(row["blocked"]),
                row["dynamic_probability"], row["status"],
                int(row["independent_observation_count"]), int(row["unique_node_count"]),
                1 + int(row["prototype_2"] is not None)))
        results.sort(key=lambda value: value.confidence, reverse=True)
        return results

    def resolve_object_relation(self, map_id: str, query: str,
                                robot_position: Sequence[float], road_tangent: Sequence[float],
                                min_confidence: float = 0.3,
                                now: Optional[float] = None) -> Optional[RememberedObject]:
        remembered = self.query_objects(map_id, min_confidence=min_confidence, now=now)
        semantic = [SemanticObject(obj.object_id, obj.label, obj.position, obj.confidence)
                    for obj in remembered]
        match = resolve_relation(query, semantic, robot_position, road_tangent)
        if match is None:
            return None
        return next(obj for obj in remembered if obj.object_id == match.object_id)

    def query_memory_goal(self, map_id: str, label: str,
                          min_confidence: float = 0.6,
                          now: Optional[float] = None) -> Optional[RememberedObject]:
        """Return the best confirmed object that has a reusable approach/view pose."""
        candidates = self.query_objects(
            map_id, label=label, min_confidence=min_confidence, now=now,
            confirmed_only=True)
        candidates = [item for item in candidates
                      if item.approach_pose is not None or item.best_view_pose is not None]
        candidates.sort(
            key=lambda item: (
                item.blocked,
                -item.confidence,
                -item.association_confidence,
                -item.observation_count,
            ))
        return candidates[0] if candidates else None

    def confirmed_heading_bonus(self, map_id: str, label: str, node_uuid: str,
                                heading_bin_count: int = 16) -> np.ndarray:
        """Return a bounded directional prior from confirmed object views only."""
        count = max(1, int(heading_bin_count))
        bonus = np.zeros(count, dtype=np.float32)
        rows = self._db.execute("""
          SELECT * FROM semantic_objects
          WHERE map_id=? AND label=? AND status='CONFIRMED'
            AND (best_view_node_uuid=? OR approach_node_uuid=? OR anchor_node_uuid=?)
        """, (map_id, label, node_uuid, node_uuid, node_uuid)).fetchall()
        for row in rows:
            anchors = (
                (row["best_view_node_uuid"], row["best_view_heading_bin"], 1.0),
                (row["approach_node_uuid"], row["approach_heading_bin"], 0.8),
                (row["anchor_node_uuid"], row["anchor_heading_bin"], 0.5),
            )
            strength = float(np.clip(
                float(row["confidence"]) * float(row["association_confidence"]),
                0.0, 1.0))
            for anchor_node, anchor_bin, weight in anchors:
                if anchor_node != node_uuid or anchor_bin is None:
                    continue
                index = int(anchor_bin) % count
                bonus[index] = max(bonus[index], strength * weight)
                bonus[(index - 1) % count] = max(
                    bonus[(index - 1) % count], strength * weight * 0.5)
                bonus[(index + 1) % count] = max(
                    bonus[(index + 1) % count], strength * weight * 0.5)
        return bonus

    def mark_object(self, object_id: str, *, reached: Optional[bool] = None,
                    blocked: Optional[bool] = None, status: Optional[str] = None):
        values = {}
        if reached is not None:
            values["reached"] = int(bool(reached))
        if blocked is not None:
            values["blocked"] = int(bool(blocked))
        if status is not None:
            values["status"] = str(status)
        if not values:
            return
        clause = ",".join(f"{key}=?" for key in values)
        self._db.execute(
            f"UPDATE semantic_objects SET {clause} WHERE object_id=?",
            (*values.values(), object_id))
        self._db.commit()

    def record_search_coverage(self, map_id: str, query: str, node_uuid: str,
                               heading_bins: Iterable[int], negative_evidence: float,
                               timestamp: Optional[float] = None):
        now = float(time.time() if timestamp is None else timestamp)
        mask = 0
        for index in heading_bins:
            if int(index) >= 0:
                mask |= 1 << int(index)
        old = self._db.execute(
            "SELECT * FROM search_coverage WHERE map_id=? AND query=? AND node_uuid=?",
            (map_id, query, node_uuid)).fetchone()
        if old is None:
            self._db.execute("""
              INSERT INTO search_coverage(map_id,query,node_uuid,heading_mask,negative_evidence,
                observation_count,first_searched,last_searched) VALUES(?,?,?,?,?,?,?,?)
            """, (map_id, query, node_uuid, mask, float(negative_evidence), 1, now, now))
        else:
            self._db.execute("""
              UPDATE search_coverage SET heading_mask=?,negative_evidence=?,
                observation_count=?,last_searched=? WHERE map_id=? AND query=? AND node_uuid=?
            """, (int(old["heading_mask"]) | mask,
                  max(float(old["negative_evidence"]), float(negative_evidence)),
                  int(old["observation_count"]) + 1, now, map_id, query, node_uuid))
        self._db.commit()

    def get_search_coverage(self, map_id: str, query: str, node_uuid: str):
        row = self._db.execute(
            "SELECT * FROM search_coverage WHERE map_id=? AND query=? AND node_uuid=?",
            (map_id, query, node_uuid)).fetchone()
        return None if row is None else dict(row)

    def search_penalty(self, map_id: str, query: str, node_uuid: str,
                       heading_bin_count: int = 16) -> float:
        coverage = self.get_search_coverage(map_id, query, node_uuid)
        if coverage is None:
            return 0.0
        observed = int(coverage["heading_mask"]).bit_count()
        ratio = observed / max(int(heading_bin_count), 1)
        return float(coverage["negative_evidence"]) * ratio

    def record_traversal(self, map_id: str, from_node: str, to_node: str,
                         outcome: int, robot_profile_id: str = "default",
                         start_heading_bin: Optional[int] = None,
                         end_heading_bin: Optional[int] = None,
                         duration: Optional[float] = None,
                         commanded_distance: Optional[float] = None,
                         observed_distance: Optional[float] = None,
                         slip_ratio: Optional[float] = None,
                         energy_cost: Optional[float] = None,
                         timestamp: Optional[float] = None):
        now = float(time.time() if timestamp is None else timestamp)
        self._db.execute("""
          INSERT INTO traversal_history(map_id,from_node,to_node,robot_profile_id,
            start_heading_bin,end_heading_bin,outcome,duration,commanded_distance,
            observed_distance,slip_ratio,energy_cost,observed_at)
          VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)
        """, (map_id, from_node, to_node, robot_profile_id,
              start_heading_bin, end_heading_bin, int(outcome), duration,
              commanded_distance, observed_distance, slip_ratio, energy_cost, now))
        self._db.commit()

    def traversal_stats(self, map_id: str, from_node: str, to_node: str,
                        success_outcome: int = 1,
                        start_heading_bin: Optional[int] = None,
                        end_heading_bin: Optional[int] = None):
        sql = """SELECT outcome,duration,observed_distance,slip_ratio,energy_cost
                 FROM traversal_history WHERE map_id=? AND from_node=? AND to_node=?"""
        args = [map_id, from_node, to_node]
        if start_heading_bin is not None:
            sql += " AND start_heading_bin=?"
            args.append(int(start_heading_bin))
        if end_heading_bin is not None:
            sql += " AND end_heading_bin=?"
            args.append(int(end_heading_bin))
        rows = self._db.execute(sql, args).fetchall()
        attempts = len(rows)
        successes = sum(int(row["outcome"]) == int(success_outcome) for row in rows)
        mean = lambda key: (sum(float(row[key]) for row in rows if row[key] is not None) /
                            max(sum(row[key] is not None for row in rows), 1))
        return {
            "attempts": attempts,
            "successes": successes,
            "success_probability": (successes + 1.0) / (attempts + 2.0),
            "mean_duration": mean("duration"),
            "mean_distance": mean("observed_distance"),
            "mean_slip_ratio": mean("slip_ratio"),
            "mean_energy": mean("energy_cost"),
        }

    def close(self):
        self._db.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
