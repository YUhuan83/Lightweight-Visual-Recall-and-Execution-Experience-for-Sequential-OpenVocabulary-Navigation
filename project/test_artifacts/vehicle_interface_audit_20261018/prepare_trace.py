"""Diagnostic namespace copy only; preserve production algorithms and thresholds."""
import difflib,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent/'trace'
changes={
    'if (!enough_time(!feasible.empty())) { break; }':
    'if (!enough_time(!feasible.empty())) { RCLCPP_WARN(_logger,"TRACE no_time remaining=%.9f required=%.9f",std::chrono::duration<double>(deadline-Clock::now()).count(),_max_planning_time); break; }',
    'catch (const std::runtime_error &) { continue; }':
    'catch (const std::runtime_error & e) { RCLCPP_WARN(_logger,"TRACE suffix_exception: %s",e.what()); continue; }',
    'if (suffix.poses.empty()) { continue; }':
    'if (suffix.poses.empty()) { RCLCPP_WARN(_logger,"TRACE empty_suffix"); continue; }',
    'auto first = poseOf(suffix.poses.front()), end = c.poses.back();':
    'auto first = poseOf(suffix.poses.front()), end = c.poses.back(); RCLCPP_WARN(_logger,"TRACE join pos=%.12g yaw=%.12g",std::hypot(first.x-end.x,first.y-end.y),std::abs(std::remainder(first.yaw-end.yaw,2.*std::acos(-1.))));',
    'if (curvatureFeasible(combined,resolution_,limits_.radius,limits_.bins) && validate(result)) {':
    'const bool trace_geometry=curvatureFeasible(combined,resolution_,limits_.radius,limits_.bins); RCLCPP_WARN(_logger,"TRACE combined geometry=%d remaining=%.9f",trace_geometry,std::chrono::duration<double>(deadline-Clock::now()).count()); if (trace_geometry && validate(result)) {',
    'if (cost ==': 'if (cost ==',
}
manifest={};diffs=[]
for sub in ('src/connector.cpp','src/planner.cpp','src/path_contract.cpp','include/wildos_ackermann_smac/connector.hpp','include/wildos_ackermann_smac/planner.hpp','include/wildos_ackermann_smac/path_contract.hpp'):
    source=ROOT/'ackermann_smac'/sub;before=source.read_text();after=before
    if sub=='src/planner.cpp':
        for a,b in changes.items():
            if a==b:continue
            assert after.count(a)==1,(a,after.count(a));after=after.replace(a,b)
        old='if(!geometry_ok(raw) || !validate(raw)) {'
        assert after.count(old)==1
        after=after.replace(old,'const bool trace_raw_geometry=geometry_ok(raw); RCLCPP_WARN(_logger,"TRACE raw poses=%zu geometry=%d remaining=%.9f",raw.poses.size(),trace_raw_geometry,std::chrono::duration<double>(deadline-Clock::now()).count()); if(!trace_raw_geometry || !validate(raw)) {')
    after=after.replace('wildos_ackermann_smac','wildos_ackermann_smac_trace')
    dst=OUT/sub.replace('wildos_ackermann_smac','wildos_ackermann_smac_trace');dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(after)
    manifest[sub]=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),trace_sha256=hashlib.sha256(dst.read_bytes()).hexdigest())
    diffs.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile=sub,tofile=str(dst)))
(OUT/'trace_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(OUT/'diagnostic.diff').write_text(''.join(diffs))
print(json.dumps(manifest,indent=2))
