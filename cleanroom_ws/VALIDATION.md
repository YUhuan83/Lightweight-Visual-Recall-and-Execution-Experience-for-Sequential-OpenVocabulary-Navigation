# Validation scope — 2026-11-03

This is a source checkpoint published at the user's explicit request. Full
church search acceptance has **not passed**. Publishing this checkpoint does
not promote the isolated candidate to the server's default installation.

## Changes

- Mapper checks new and existing frontier exits against historical explored
  radii across the graph. The 4 m node attachment radius no longer limits this
  check; covered exits outside the current local map can retire.
- Exploration candidate checks and generated routes use the same selected
  owner and measured exit. Semantic goals retain the existing goal planner.
- Includes earlier frontier reselection, removed-exit checks, exploration
  failure feedback, rolling route intent, and passive recall corrections.
- Vehicle geometry, collision checks, sensors, Smac/RPP, and arrival thresholds
  were not relaxed by the latest frontier repair.

## Verified scope

- Mapper: 2 public-API tests passed; both fail with the preceding component.
- Planner: 8 C++ cases passed. Real ROS frontier handoff: 2 passed.
- ROS actions/confirmation/coordinates/readiness: 4 passed; navigation: 20 passed.
- Both modified C++ packages built without warnings.
- Publication copy: Python syntax parsing and 6 memory/recorded-recall tests passed.
- Four recorded graph snapshots were processed by the real Mapper component:
  covered exits remaining changed from 11/18/28/25 to zero, while other exits
  remained. A remote synthetic map triggered cleanup; this is not a complete
  historical sensor replay.

## Field trial 20261103_183850

Town gas-station spawn; `church`; 300.260 s search, with 36.786 s startup
excluded; real vehicle physics; no video. Both GPUs were heavily loaded, so
different-route runs do not constitute a controlled performance A/B.

- Travel 231.911 m; net displacement 46.229 m; maximum departure 46.236 m.
- Near-stop fraction 9.655%; longest near-stop 14.363 s; no odometry gaps >1 s.
- Of 77 matched exploration path snapshots, none ended >0.2 m inside recorded
  historical explored spheres. This is sampling, not an exhaustive proof.
- No church mask, 3D target, confirmation, or verified arrival was produced.
- At about 286 s, RPP aborted on a command-arc collision. Smac rejected later
  candidates until the deadline. The source of that collision is unresolved.
- Harness closed with `errors=[]`; SIGINT shutdown still produced child-process
  exceptions. Full navigation, recovery, and clean-shutdown acceptance remain
  incomplete. The longest stop worsened compared with the preceding trial.

Raw bags, videos, models, credentials, and per-run artifacts are excluded.
Publication text uses LF line endings; runtime source semantics are preserved.
