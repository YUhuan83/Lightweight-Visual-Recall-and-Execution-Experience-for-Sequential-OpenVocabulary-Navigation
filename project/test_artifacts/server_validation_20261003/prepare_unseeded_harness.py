"""Generate a separate trial without the historical station-seeding stage."""
from pathlib import Path

root = Path(__file__).resolve().parent
source = root.parent / 'server_validation_20261002/full_town_memory_test.py'
text = source.read_text(encoding='utf-8')
text = text.replace('observe station, search house, return via memory', 'search house first, then station without prior station instructions')
text = text.replace("self.phase = 'station_observation'", "self.phase = 'house_search'")
text = text.replace("self.query = 'gas station'", "self.query = 'house'")
text = text.replace("if not Path('/proc', str(vision_pid)).exists():", "if not Path('/proc', str(vision_pid), 'status').exists() or 'State:\\tZ' in Path('/proc', str(vision_pid), 'status').read_text():")
start = text.index("    n.change_query('gas station')\n    last_replay")
end = text.index("    n.set_bool('/wildos_cmd_vel_to_control', 'enable', True)", start)
text = text[:start] + "    report['station_views_before_departure'] = []\n    report['station_preseeded'] = False\n" + text[end:]
text = text.replace("    if not house_found:\n        raise RuntimeError('House was not confirmed; full multi-goal sequence cannot proceed')", "    if not house_found:\n        report['second_phase_is_diagnostic'] = True\n        n.event('diagnostic_switch_after_house_failure')\n    else:\n        report['second_phase_is_diagnostic'] = False")
start = text.index("        known = report['station_views_before_departure']")
end = text.index("        if (matches_region", start)
text = text[:start] + "        matches_region = bool(confirmed and confirmed.get('search_confirmed') is True)\n" + text[end:]
text = text.replace("returned=returned, return_elapsed", "gas_station_found=returned, returned=returned, return_elapsed")
text = text.replace("    n.event('full_test_finished'", "    station_candidates = [e['payload']['views'] for e in n.events if e['kind'] == 'memory_candidates' and e['payload']['query'] == 'gas station']\n    report['station_candidates_at_switch'] = station_candidates[0] if station_candidates else None\n    report['queries_sent'] = [e['query'] for e in n.events if e['kind'] == 'query_sent']\n    n.event('full_test_finished'")
(root / 'unseeded_house_station_test.py').write_text(text, encoding='utf-8')
runner = (root / 'run_trial.py').read_text(encoding='utf-8').replace("ROOT/'full_town_memory_test.py'", "ROOT/'unseeded_house_station_test.py'")
(root / 'run_unseeded_trial.py').write_text(runner, encoding='utf-8')
