# Offline Contract v1

This package is independent of ROS, BeamNGpy and the original model source.
The validators in `core.py` are normative and reject unknown fields in calibration,
frame, timing, transform and binding objects. No aliases or implicit frame renames.
`load_query()` validates the upstream report fields it consumes; upstream diagnostic
fields are intentionally allowed. Never supply target GT or hidden semantics.

## Integration API

```python
from target_localization import Localizer, Policy, load_query, read_json

query = load_query("/acquired/query-output/result.json", "query_000")
calibration = read_json("/validated/calibration.json")
engine = Localizer(calibration, Policy())
# frame is the exact structure below. API points_xyz_m is finite Nx3 numeric
# rows or an ndarray, not a filename. Keep this engine for the acquired stream.
handoff = engine.process(query, frame)
# When no image arrives, feed a real normalized acquisition clock to expire TTL:
handoff = engine.tick(now)
```

`query` must come from `load_query`, not invented model results. `localize(query,
frame, calibration, policy)` is the stateless geometry API and raises `Invalid`
on invalid contracts; `Localizer.process` instead releases evidence and emits
`waiting` with unresolved reasons. Direct API callers are responsible for capture
file integrity; CLI hashes the supplied image file. Cryptographic hashes bind
bytes, not the truthfulness of acquisition metadata or semantic correctness.

## Vocabulary Association

Inspected `vocabulary_query/query.py` lines 75-96, 172-177, 193-218:

- Required `result.json`: `status="completed"`, `inference_ran=true`,
  `image_size=[width,height]`, `input_size=[resized_width,resized_height]`,
  finite `threshold`, exact `box_convention`, and `results` selected by unique prefix.
- Exact box convention: `xyxy, exclusive right/bottom, EXIF-oriented original image pixels`.
- Each selected result has `query`, `prefix`, `raw_min`, `raw_max`, `regions`.
- Each region has `box_xyxy`, `area_pixels`, `mean_cosine`, `max_cosine`.
- Files: `<prefix>_cosine.npy` (finite HxW raw cosine in [-1,1]) and
  `<prefix>_region_mask.png` (0/255 gray8 PNG). No PIL/OpenCV dependency.
- Both maps are full original EXIF-oriented resolution, although model inference
  used `input_size`. No resizing, multiplying boxes by scale or using heatmap colors.
- Reconstruct raw-threshold 8-connected components; kept mask must retain whole
  components. Match every retained component to exactly one reported geometry/area/
  score tuple, not to arbitrary label IDs or report list index. Verify sorted maxima.
  Float32 summary comparisons have fixed absolute 1e-6 tolerance, no relative tolerance;
  input finiteness, cosine bounds, dimensions, row lengths and timestamps are exact.
- Region mask is a union, NOT instance segmentation. Multiple surviving 3D candidates
  cause abstention. Cosine is not a calibrated object confidence or probability.

## Calibration

Exact top-level fields:
`schema status evidence world_frame camera_frame local_cloud_frame time_source units camera local_to_camera`.

`schema="target-calibration/1"`, runtime `status="VERIFIED"`, `units="m"`.
All frame and clock names are explicit nonempty strings. The three frame names are
distinct. `world_frame` must be a stable metric reference within the epoch. An
independent estimator's stable local odometry frame is acceptable; geographic or
simulator-global truth is not required. Relocalization/origin changes require a new epoch.

Every `evidence` has exactly `accepted` (must be true) and nonempty `reference`.
These are externally reviewed measurement/provenance references, not permission
switches. The shipped draft intentionally has unknowns and false acceptance.

`camera` exact fields:
`width height projection pixel_center axes pixels_in_target_optical_frame K D evidence`.

- Positive integer dimensions refer to actual EXIF-oriented optical image pixels.
- `projection="rectilinear_pinhole"`, `axes="x_right_y_down_z_forward"`.
- `pixel_center="integer_centers_half_extent"`, `pixels_in_target_optical_frame=true`.
- `K=[[fx,0,cx],[0,fy,cy],[0,0,1]]`, positive focal lengths in pixels and principal
  point within image half-extents; `D=[0,0,0,0,0]` explicitly verified undistorted.
- Projection: `u=fx*X/Z+cx`, `v=fy*Y/Z+cy`. Valid half-open footprint:
  `-0.5 <= u < width-0.5`, `-0.5 <= v < height-0.5`. Rasterize `floor(uv+0.5)`.
- This is the same pixel convention as `beamng/adapters/core.py:162-180`.
  If independently validated adapter FOV calibration is available, its K can be
  reshaped to 3x3, preserving frame/dimensions/evidence. This package does not infer FOV.

Every SE3 object has exact fields:
`source_frame target_frame R translation_m units evidence`.
`R` is finite 3x3, right-handed orthonormal with fixed absolute 1e-9 validation,
`translation_m` finite length 3, `units="m"`. Column-vector equation:
`p_target = R @ p_source + translation_m`. No Euler inference, quaternion ordering
guess, nearest-pose matching or invented identity transform.

`local_to_camera` maps the calibrated `local_cloud_frame` into `camera_frame`.
It is an explicitly validated static rigid sensor extrinsic. A world-to-moving-camera
transform is NEVER static: it must be supplied in each acquired frame below.
This v1 calibration requires local extrinsics even for world-cloud-only operation.

## Acquired Frame

Exact top-level fields: `schema mode image cloud pose now binding`.
`schema="target-frame/1"`, `mode="estimated_pose"` or explicit `"oracle_pose"`.

Timing objects have exact fields `stamp_ns source epoch time_kind`:
nonnegative integer nanoseconds, named common normalized simulation acquisition
clock `source`, nonempty epoch, `time_kind="acquisition"`.
Image, cloud and pose timing objects must be identical. No epsilon synchronization,
poll-time fallback, interpolation, scan deskew, timestamp synthesis or inferred epochs.
Caller must validate scan temporal semantics; a scan end stamp alone does not prove
all returns correspond to the image/pose time. `now` uses the same clock and epoch;
`0 <= now.stamp_ns-image.stamp_ns <= max_age_ns`.

`image` fields: `timing frame width height capture_id sha256 provenance`.
Frame/dimensions must match calibration and query full resolution. `capture_id` is
a unique acquired image ID, `sha256` is 64 lowercase hexadecimal characters,
`provenance="sensor_measurement"`.

`cloud` fields: `timing frame coordinates units provenance points_xyz_m`.
`coordinates="world"` requires `frame=world_frame`; `coordinates="sensor_local"`
requires `frame=local_cloud_frame`; `units="m"`, `provenance="sensor_measurement"`.
Input is genuine measured XYZ, finite Nx3, including explicit ndarray shape `(0,3)`
for no depth. Ragged, bool, string, NaN/Inf, extra columns and nonfinite rows are
rejected, not silently filtered. World points are numerically transformed using
the acquired world-to-camera SE3. Local points first use local-to-camera, then the
inverse acquired world-to-camera for stable physical 3D temporal comparison.

`pose` fields: `timing provenance world_to_camera`.
`world_to_camera` is SE3 from world_frame to camera_frame at the acquisition stamp.
`estimated_pose` requires `provenance="independent_estimate"`; no specific estimator
dependency or unsupported claim of validated LIO. `oracle_pose` requires exactly
`provenance="ego_truth"`; only ego pose may be GT. GT targets/clouds/semantics are
forbidden in either mode. `algorithm_mode/contract.py` likewise isolates ego truth
in oracle mode, but its `estimated_pose` profile specifically names LIO. Do not
claim an arbitrary estimate satisfies that separate LIO profile. This is a new
localization observation contract, not an alias of its navigation observation.

`binding` fields: `evidence image_sha256 artifact_sha256 query prefix`.
The image digest, exact query string and prefix must match this frame/query.
`artifact_sha256` has exactly `result cosine mask` with the digests returned by
`load_query()`. Evidence must establish that those query artifacts actually came
from this captured image; upstream result.json does not supply acquisition timing
or a source image digest. A path string alone is insufficient. No automatic binding
from smoke filenames, result modification times or model output order.

## Manifest and Handoff

Manifest exact fields: `schema status unresolved calibration policy frames`.
`schema="target-offline-manifest/1"`; `DRAFT` always waits. `VERIFIED` requires an
empty unresolved list, a verified calibration JSON path, policy object (subset of
`Policy` constructor fields), and frame entries. Paths are relative to the manifest
unless absolute. Frame entries have exactly `query_result prefix image cloud frame`.
`frame` is the acquired sidecar JSON above, with `cloud.points_xyz_m=null` only in
CLI input; CLI fills it from the explicit `cloud` npy artifact. It never fills stamps,
poses, evidence or calibration. Each entry is one actual acquisition; repeated image
digest or capture ID cannot count as another confirmation. CLI has no force flag.

`coarse3d-observation/1` contains valid candidate surface medians, per-axis MAD,
extent, unique pixel/return counts, raw mean cosine, silhouette and rejection reasons.
Median is a visible-surface estimate, NOT a geometric object center. MAD is not
calibrated pose covariance. No centroid is an approach point.

State: `SEARCHING -> TENTATIVE -> CONFIRMED` after three distinct timestamps and
distinct captures within TTL for the same exact query, mode, world frame, clock and
epoch. All medians must be within `proximity_m` of a fixed anchor (no drifting chain).
No match breaks tentative evidence. Ambiguous/missing observations never add hits.
Confirmed estimates can be retained through missing observations only until TTL;
call `tick` during silence. Invalid input immediately releases the estimate.
Query/mode/frame/epoch changes start fresh; duplicate/backward stamps reject and
release, keeping a watermark until newer time or an explicit new stream epoch.
Duplicate image hashes are conservatively rejected even if a stationary real camera
legitimately produced identical bytes. The caller must not fabricate changes.

`target-handoff/1` always has `executable=false`, `approach_pose=null`,
`path_reachability_validated=false`, `ros_publication_allowed=false`.
Only CONFIRMED exposes `target_estimate`, itself `executable=false`; tentative
geometry remains diagnostic under `observation`. There is no goal publication,
safe parking, steering, random exploration or reachable-path computation here.
