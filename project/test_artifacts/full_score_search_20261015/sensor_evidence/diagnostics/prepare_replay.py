from pathlib import Path
p=Path('/home/ly/full_score_search_20261015')
s=Path('/home/ly/search_mechanism_audit_20261015/extract_inputs.py').read_text().replace('/home/ly/wildos-console/results/session_20261015_151859','/home/ly/wildos-console/audit_results/session_20261015_163334').replace("out=Path('/home/ly/search_mechanism_audit_20261015')", "out=Path('/home/ly/full_score_search_20261015')").replace('(cases[0],cases[-1])','(cases[0],cases[len(cases)//2],cases[-1])')
(p/'extract_inputs.py').write_text(s)
