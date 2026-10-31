"""Fixed-size, query-independent visual descriptors anchored by observed LiDAR.

No images, complete feature maps or labels are retained. Retrieval yields
unconfirmed search hints; it never emits an object confirmation.
The index contains historical descriptors; semantic eligibility rules are
retrieval policy, not an additional kind of memory.
"""
import math
import numpy as np


class PassiveLandmarkIndex:
    def __init__(self, capacity=1024, max_bytes=8*1024*1024, voxel_size=1.5,
                 min_distance=1., min_angle=math.pi/12, min_interval=.5):
        self.requested_capacity = max(1, int(capacity))
        self.max_bytes = max(1024, int(max_bytes))
        self.voxel_size = float(voxel_size)
        if not math.isfinite(self.voxel_size) or self.voxel_size <= 0:
            raise ValueError('invalid semantic voxel size')
        self.min_distance, self.min_angle, self.min_interval = min_distance, min_angle, min_interval
        self.arrays = {}
        self.count = self.serial = 0
        self.last_pose = None
        self.frame = None
        self.blocked = {}

    @property
    def nbytes(self):
        return sum(a.nbytes for a in self.arrays.values())

    def should_capture(self, position, heading, now):
        if self.last_pose is None:
            return True
        previous, yaw, when = self.last_pose
        angle = abs(math.atan2(math.sin(heading-yaw), math.cos(heading-yaw)))
        return now-when >= self.min_interval and (
            math.dist(position, previous) >= self.min_distance or angle >= self.min_angle)

    def _allocate(self, dimension):
        specs = {'features': ((dimension,), np.float16), 'position': ((3,), np.float32),
                 'observer': ((4,), np.float32), 'key': ((4,), np.int32),
                 'stamp': ((), np.int64), 'last_seen': ((), np.float64),
                 'count': ((), np.uint32), 'id': ((), np.uint64),
                 'support': ((), np.uint16), 'fov': ((), np.float32)}
        per_record = sum(np.prod(shape, dtype=int)*np.dtype(dtype).itemsize
                         for shape, dtype in specs.values())
        capacity = min(self.requested_capacity, self.max_bytes//int(per_record))
        if capacity < 1:
            raise ValueError('semantic memory byte budget cannot fit one descriptor')
        self.arrays = {name: np.zeros((capacity, *shape), dtype=dtype)
                       for name, (shape, dtype) in specs.items()}

    def observe(self, features, cameras, world_points, position, heading,
                stamp_ns, now, frame, inverted=False, min_points=8):
        """Capture coherent depth surfaces per feature patch, independent of query."""
        features = np.asarray(features)
        world = np.asarray(world_points, dtype=float).reshape(-1, 3)
        if (features.ndim != 4 or features.shape[0] != len(cameras)
                or not np.all(np.isfinite(features)) or not np.all(np.isfinite(position))
                or not math.isfinite(heading) or not math.isfinite(now) or stamp_ns <= 0
                or not frame or (self.frame is not None and self.frame != frame)):
            raise ValueError('invalid passive scene observation')
        if not self.should_capture(position, heading, now):
            return 0
        world = world[np.all(np.isfinite(world), axis=1)]
        if not len(world):
            return 0
        if not self.arrays:
            self._allocate(features.shape[1])
        if features.shape[1] != self.arrays['features'].shape[1]:
            raise ValueError('semantic feature dimension changed')
        self.frame = frame
        proposals = {}
        for image, cam in zip(features, cameras):
            k, rotation = np.asarray(cam['K']), np.asarray(cam['R_wc'])
            translation = np.asarray(cam['t_wc']).reshape(3)
            height, width = int(cam['height']), int(cam['width'])
            if (k.shape != (3, 3) or rotation.shape != (3, 3) or height <= 0 or width <= 0
                    or not np.all(np.isfinite(k)) or not np.all(np.isfinite(rotation))
                    or not np.all(np.isfinite(translation)) or k[0, 0] <= 0 or k[1, 1] <= 0):
                raise ValueError('invalid passive camera calibration')
            optical = (world-translation) @ rotation
            valid = optical[:, 2] > .1
            camera_points, world_visible = optical[valid], world[valid]
            pixels = camera_points @ k.T
            pixels = pixels[:, :2] / pixels[:, 2:]
            visible = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width)
                       & (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
            camera_points, world_visible, pixels = camera_points[visible], world_visible[visible], pixels[visible]
            if not len(pixels):
                continue
            # One nearest return per projected pixel; duplicated LiDAR points
            # must not satisfy a minimum geometric support requirement.
            order = np.argsort(camera_points[:, 2])
            integer_pixels = pixels.astype(int)
            _, unique = np.unique(integer_pixels[order, 1]*width+integer_pixels[order, 0], return_index=True)
            order = order[unique]
            camera_points, world_visible, pixels = camera_points[order], world_visible[order], pixels[order]
            fh, fw = image.shape[1:]
            px = np.minimum(fw-1, (pixels[:, 0]*fw/width).astype(int))
            py = np.minimum(fh-1, (pixels[:, 1]*fh/height).astype(int))
            # Pool geometric support over 2x2 feature patches: a distant
            # facade can have too few LiDAR pixels in one native patch.
            # Its descriptor still comes from the median supported pixel,
            # preserving the model's feature space rather than compressing D.
            group_width = (fw+1)//2
            patch_ids = (py//2)*group_width+(px//2)
            for patch in np.unique(patch_ids):
                ids = np.flatnonzero(patch_ids == patch)
                ids = ids[np.argsort(camera_points[ids, 2])]
                depths = camera_points[ids, 2]
                cuts = np.flatnonzero(np.diff(depths) > np.maximum(.6, .04*depths[:-1]))+1
                surfaces = [v for v in np.split(ids, cuts)
                            if len(v) >= min_points and len(v)/len(ids) >= .2]
                if not surfaces:
                    continue
                ids = surfaces[0]
                centre = np.median(world_visible[ids], axis=0)
                x = min(fw-1, int(np.median(pixels[ids, 0])*fw/width))
                y = min(fh-1, int(np.median(pixels[ids, 1])*fh/height))
                if inverted:
                    y, x = fh-1-y, fw-1-x
                vector = image[:, y, x].astype(np.float32)
                norm = np.linalg.norm(vector)
                if norm < 1e-8:
                    continue
                vector /= norm
                key = (*np.floor(centre/self.voxel_size).astype(int),
                       round(heading/(math.pi/4)) % 8)
                # Multiple image patches at one physical location contribute
                # one record and one independent observation, not fake frames.
                if key not in proposals or len(ids) > proposals[key][2]:
                    proposals[key] = (centre, vector, len(ids),
                                      float(2*np.arctan(width/(2*k[0, 0]))))
        for key, (centre, vector, support, fov) in proposals.items():
            self._insert(key, centre, vector, position, heading, stamp_ns, now, support, fov)
        if proposals:
            self.last_pose = (tuple(position), heading, now)
        return len(proposals)

    def _insert(self, key, point, vector, position, heading, stamp, now, support, fov):
        a = self.arrays
        matches = np.flatnonzero(np.all(a['key'][:self.count] == key, axis=1))
        index = next((int(i) for i in matches if float(a['features'][i].astype(np.float32) @ vector) >= .8), None)
        if index is None:
            if self.count < len(a['id']):
                index = self.count
                self.count += 1
            else:
                # Evict a redundant near neighbour first. Recency alone would
                # erase isolated landmarks at the spawn during a long search.
                distances = np.linalg.norm(a['position'][:self.count]-point, axis=1)
                redundant = np.flatnonzero(distances < 2*self.voxel_size)
                if not len(redundant):
                    _, inverse, counts = np.unique(np.floor(a['position'][:self.count]/(2*self.voxel_size)),
                                                   axis=0, return_inverse=True, return_counts=True)
                    redundant = np.flatnonzero(counts[inverse] > 1)
                pool = redundant if len(redundant) else np.arange(self.count)
                utility = np.minimum(a['count'][pool], 20).astype(float)
                utility += np.minimum(now-a['last_seen'][pool], 600)*(-.002)
                index = int(pool[np.argmin(utility)])
            self.serial += 1
            a['id'][index], a['count'][index] = self.serial, 0
            a['features'][index] = vector
            a['position'][index] = point
        elif a['stamp'][index] == stamp:
            return
        else:
            weight = min(int(a['count'][index]), 9)
            fused = a['features'][index].astype(np.float32)*weight+vector
            a['features'][index] = fused/max(np.linalg.norm(fused), 1e-8)
            a['position'][index] = (a['position'][index]*weight+point)/(weight+1)
        a['key'][index] = key
        a['observer'][index] = [*position, heading]
        a['stamp'][index], a['last_seen'][index] = stamp, now
        a['count'][index] += 1
        a['support'][index], a['fov'][index] = min(support, 65535), fov

    def retrieve(self, query, text_feature, threshold, now, limit=4):
        if not self.count:
            return []
        text = np.asarray(text_feature, dtype=np.float32).reshape(-1).copy()
        if (len(text) != self.arrays['features'].shape[1] or not np.all(np.isfinite(text))
                or np.linalg.norm(text) < 1e-8):
            raise ValueError('invalid semantic text feature')
        text /= np.linalg.norm(text)
        a = self.arrays
        scores = a['features'][:self.count].astype(np.float32) @ text
        eligible = np.flatnonzero(scores > threshold)
        used, result = set(), []
        for seed in eligible[np.argsort(scores[eligible])[::-1]]:
            if int(seed) in used:
                continue
            neighbours = eligible[np.linalg.norm(a['position'][eligible]-a['position'][seed], axis=1) <= 3.]
            neighbours = np.asarray([i for i in neighbours if int(i) not in used], dtype=int)
            centre = np.median(a['position'][neighbours], axis=0)
            neighbours = neighbours[np.linalg.norm(a['position'][neighbours]-centre, axis=1) <= 3.]
            # Require spatial support from distinct voxels, rather than one
            # weak patch or several views of exactly the same pixel location.
            if len(np.unique(a['key'][neighbours, :3], axis=0)) < 2:
                continue
            centre = np.median(a['position'][neighbours], axis=0)
            radius = min(3., max(.25, float(np.max(np.linalg.norm(a['position'][neighbours, :2]-centre[:2], axis=1)))))
            identity = 'passive:%s:%s' % (query, int(np.min(a['id'][neighbours])))
            used.update(map(int, neighbours))
            if self.blocked.get(identity, 0) > now:
                continue
            observer = a['observer'][seed]
            result.append(dict(id=identity, frame=self.frame, position=observer[:3].astype(float).tolist(),
                               heading=float(observer[3]), object_position=centre.astype(float).tolist(),
                               object_radius=radius, horizontal_fov=float(a['fov'][seed]),
                               observed_at=float(a['last_seen'][seed]),
                               observation_stamp_ns=int(a['stamp'][seed]),
                               evidence='passive_semantic_landmarks', search_confirmed=False,
                               semantic_similarity=float(scores[seed]), support_landmarks=len(neighbours)))
            if len(result) >= limit:
                break
        # Bounded feedback state; object identities disappear as their slots
        # are evicted and a new serial is allocated.
        self.blocked = {k:v for k,v in self.blocked.items() if v > now}
        return result

    def reject(self, identity, now, cooldown=30.):
        if len(self.blocked) >= self.requested_capacity:
            self.blocked.pop(next(iter(self.blocked)))
        self.blocked[identity] = now+max(0., cooldown)
