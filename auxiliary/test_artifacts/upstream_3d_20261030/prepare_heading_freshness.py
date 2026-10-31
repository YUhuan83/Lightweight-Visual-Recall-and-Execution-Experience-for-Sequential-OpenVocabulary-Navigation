"""Align existing heading gates with the existing physical-assessment TTL."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
parent=Path('/home/ly/execution_continuity_review_20261022')
base=parent/'v70_upstream_arrival_120s';new=parent/'v71_heading_freshness'
relative=Path('wildos_port/repro_check/src/nebula2-wildos/graphnav_planner')
if '--apply' not in sys.argv:
    assert not new.exists();new.mkdir()
    shutil.copytree(base/'candidate',new/'candidate',symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
    (new/'full').mkdir()
    for p in (base/'full').iterdir():
        if p.is_file() and p.suffix in ('.py','.json'):(new/'full'/p.name).write_text(p.read_text().replace(str(base),str(new)))
    (new/'full/candidate').symlink_to('../candidate',target_is_directory=True)
    config=json.loads((new/'full/config.json').read_text())
    config['setups']=[str(base/'install/local_setup.bash') if x==str(new/'install/local_setup.bash') else x for x in config['setups']]
    config['setups'].append(str(new/'install/local_setup.bash'))
    (new/'full/config.json').write_text(json.dumps(config,indent=2)+'\n')
    lines=(base/'run_full.sh').read_text().splitlines();last=max(i for i,x in enumerate(lines) if x.startswith('source '))
    lines.insert(last+1,f'source {new}/install/local_setup.bash')
    (new/'run_full.sh').write_text('\n'.join(x if x.startswith('source ') else x.replace(str(base),str(new)) for x in lines)+'\n')
    for name in ('audit_fluency_120s.py','audit_target_3d.py'):shutil.copy2(base/name,new/name)
    test=new/'candidate'/relative/'test/test_frontier_endpoint.cpp'
    with test.open('a') as out:out.write('''

TEST(HeadingFreshness, OldBlockedHeadingDoesNotOutliveItsPhysicalAssessment) {
  for (int stamp : {90,94,95,100}) {
    auto g=fixture({point(3.)});
    graphnav_msgs::msg::RobotNodeTraversability a;
    a.robot_profile_id="default";a.observed_at.sec=stamp;a.confidence=.8;
    a.feasible=false;a.physical_traversability=0.;
    g->nodes[1].robot_traversability.push_back(a);
    graphnav_msgs::msg::HeadingCandidate heading;
    heading.traversable=false;heading.heading=0.;heading.predicted_utility=.8;
    g->nodes[1].heading_candidates.push_back(heading);
    graphnav_planner::Planner p(rclcpp::get_logger("heading_freshness"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    auto path=p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp));
    if(stamp<95) { ASSERT_FALSE(path.empty())<<stamp;EXPECT_DOUBLE_EQ(path.back().x(),3.); }
    else EXPECT_TRUE(path.empty())<<stamp;
  }
}

TEST(HeadingFreshness, FreshAllowedHeadingAndFreshLowConfidenceDenialKeepTheirMeaning) {
  for(bool allowed : {false,true}) {
    auto g=fixture({point(3.)});
    graphnav_msgs::msg::RobotNodeTraversability a;
    a.robot_profile_id="default";a.observed_at=g->header.stamp;
    a.confidence=allowed?.8:0.;a.feasible=allowed;a.physical_traversability=allowed?1.:0.;
    g->nodes[1].robot_traversability.push_back(a);
    graphnav_msgs::msg::HeadingCandidate h;
    h.traversable=allowed;h.heading=0.;h.predicted_utility=.8;
    g->nodes[1].heading_candidates.push_back(h);
    graphnav_planner::Planner p(rclcpp::get_logger("heading_freshness"));
    p.set_trav_class("default");p.update_graph(g);Eigen::Vector3d goal(50.,0.,0.);
    EXPECT_EQ(!p.plan_to_goal(goal,.5,rclcpp::Time(g->header.stamp)).empty(),allowed);
  }
}
''')
else:
    path=new/'candidate'/relative/'src/planner.cpp';text=path.read_text()
    old='bool assessment_fresh(const Assessment* assessment, rclcpp::Time now, double ttl)'
    assert old in text;text=text.replace(old,old[:-1]+', double minimum_confidence = 0.7)')
    text=text.replace('assessment->confidence < 0.7f ||\n      assessment->confidence > 1.0f) return false;',
                      'assessment->confidence < minimum_confidence ||\n      assessment->confidence > 1.0f) return false;')
    old='''      const auto* heading_candidate = best_heading_candidate(
          node, std::atan2(heading.y(), heading.x()));
      if (!node.heading_candidates.empty() && heading_candidate == nullptr)'''
    replacement='''      const bool heading_current = physical == nullptr ||
          assessment_fresh(physical, current_time, stale_observation_seconds_, 0.0);
      const auto* heading_candidate = heading_current ? best_heading_candidate(
          node, std::atan2(heading.y(), heading.x())) : nullptr;
      if (heading_current && !node.heading_candidates.empty() && heading_candidate == nullptr)'''
    assert old in text;path.write_text(text.replace(old,replacement))
    path=new/'candidate'/relative/'src/planner_node.cpp';text=path.read_text()
    old='      double physical_confidence = 0.0;'
    assert text.count(old)==1;text=text.replace(old,'      bool heading_current = assessment == node.robot_traversability.end();\n'+old)
    old='          physical_confidence = assessment->confidence;'
    assert text.count(old)==1;text=text.replace(old,'          heading_current = true;\n'+old)
    old='      if (!node.heading_candidates.empty())'
    assert text.count(old)==1;text=text.replace(old,'      if (heading_current && !node.heading_candidates.empty())')
    path.write_text(text)
    manifest=json.loads((base/'full/frozen_source_manifest.json').read_text());changed=[]
    for name,previous in manifest['members'].items():
        current=hashlib.sha256((new/'candidate'/name).read_bytes()).hexdigest()
        if current!=previous:changed.append(name)
        manifest['members'][name]=current
    (new/'full/frozen_source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (new/'candidate_scope.json').write_text(json.dumps(dict(
        changed=changed,no_new_runtime_components=True,
        reason='Cached heading veto must expire with its source physical assessment; fresh constraints stay enforced.',
        search_s=120,arrival='XYZ distance <3m',baseline=str(base)),indent=2))
    print((new/'candidate_scope.json').read_text())
