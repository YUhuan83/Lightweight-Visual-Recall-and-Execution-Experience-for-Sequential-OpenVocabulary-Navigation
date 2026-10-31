"""Prototype full-route comparison within the existing planning budget."""
import hashlib,json,zipfile
from pathlib import Path
root=Path(__file__).resolve().parents[2];out=Path(__file__).parent
files=[p for p in (root/'ackermann_smac').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
source=(root/'ackermann_smac/src/planner.cpp').read_text(encoding='utf8')
source=source.replace('#include <cmath>','#include <cmath>\n#include <algorithm>')
source=source.replace('>= _max_planning_time + .01;', '>= _max_planning_time + .05;')
source=source.replace('  for (const auto & c : candidates.curves) {','''  // The shortest start connector need not lead to the cheapest complete route.
  // Retain feasible alternatives under the same total/native time limits.
  std::vector<std::pair<double, nav_msgs::msg::Path>> feasible;
  for (const auto & c : candidates.curves) {''')
old='    if (curvatureFeasible(combined,resolution_,limits_.radius,limits_.bins) && validate(result)) { return result; }'
new='''    if (curvatureFeasible(combined,resolution_,limits_.radius,limits_.bins) && validate(result)) {
      double cost=0.,prev_dx=0.,prev_dy=0.; bool have_previous=false; unsigned cusps=0;
      for(size_t i=1;i<combined.size();i++) {
        const double dx=combined[i].x-combined[i-1].x,dy=combined[i].y-combined[i-1].y;
        const double length=std::hypot(dx,dy);
        if(length<=1e-9) { continue; }
        cost+=length;
        if(have_previous && dx*prev_dx+dy*prev_dy<0.) { cost+=3.;++cusps; }
        prev_dx=dx;prev_dy=dy;have_previous=true;
      }
      RCLCPP_INFO(_logger,"continuous_complete_candidate: cost=%.6f cusps=%u",cost,cusps);
      feasible.emplace_back(cost,std::move(result));
    }'''
assert old in source;source=source.replace(old,new)
old='  throw nav2_core::PlannerException("No safe bounded continuous-start plan");'
new='''  std::stable_sort(feasible.begin(),feasible.end(),[](const auto & a,const auto & b) {
    return a.first<b.first;
  });
  // A previously checked alternative can become blocked while another is
  // searched. Recheck in cost order against the latest map before returning.
  for(auto & candidate:feasible) {
    if(validate(candidate.second)) {
      RCLCPP_INFO(_logger,"continuous_complete_selected: cost=%.6f candidates=%zu",
        candidate.first,feasible.size());
      return candidate.second;
    }
  }
  throw nav2_core::PlannerException("No safe bounded continuous-start plan");'''
assert old in source;source=source.replace(old,new)
(out/'planner.candidate.cpp').write_text(source,encoding='utf8')
manifest={}
with zipfile.ZipFile(out/'complete_path_sources.zip','w',zipfile.ZIP_DEFLATED) as z:
    for mode in ('baseline','candidate'):
        for p in files:
            relative=str(p.relative_to(root)).replace('\\','/')
            data=source.encode() if mode=='candidate' and relative=='ackermann_smac/src/planner.cpp' else p.read_bytes()
            key=mode+'/'+relative
            z.writestr(key,data);manifest[key]=hashlib.sha256(data).hexdigest()
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
print(json.dumps(dict(files=len(manifest),candidate_planner=hashlib.sha256(source.encode()).hexdigest())))
