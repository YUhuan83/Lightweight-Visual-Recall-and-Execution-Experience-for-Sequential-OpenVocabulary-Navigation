# Field validation snapshot — 2026-11-02

This is the result for the source snapshot in this directory, synchronized with the server runtime before this commit.

## Checks

- Core ROS Python suite: 48 passed; 3 vision-dependent cases were skipped in the core Python environment by design.
- Vision Python: those 3 semantic-support cases passed independently.
- Ruff format/check passed.
- Core source and test hashes were compared with the server-tested files before packaging. The GitHub copy uses LF line endings; no logic changes were made during this normalization.

## Same-process mission

Town spawn; one map/process; fixed 300-second search budget after the first query, startup and shutdown excluded. Queries: `traffic light` → `bus stop shelter` → `gas station`.

- Startup: 32.020 s; search: 300.033 s; process closed cleanly; runtime errors: none.
- `traffic light` passed at 109.738 s; independent vehicle-to-target XY distance was 1.850 m.
- `bus stop shelter` began at 109.859 s, but no target candidate/confirmation was produced before the deadline. The stage-end RGB did not show a covered shelter. `gas station` did not run.
- Overall mission: **failed/incomplete**. This is not a three-target acceptance result.
- Total travel: 201.521 m; net displacement: 66.927 m; whole-run near-stop fraction: 12.03% (longest continuous near-stop 12.201 s). Exploration-only near-stop fraction before target discovery: 5.64%. These are single-run measurements, not a stability claim.

The corresponding RViz recording and full local evidence are held outside the source snapshot; no video, ROS bag, model weights, map archive, or credentials are included here.
