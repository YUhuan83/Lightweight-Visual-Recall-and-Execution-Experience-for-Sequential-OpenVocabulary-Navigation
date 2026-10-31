"""Select the main RViz window, excluding Qt/Ogre child render windows."""
import re

def select_rviz_window(tree):
    candidates=[]
    for line in tree.splitlines():
        match=re.match(r'\s*(0x[0-9a-fA-F]+)\s+".*":\s+\("rviz2"\s+"rviz2"\)\s+(\d+)x(\d+)',line)
        if match and int(match[2])>=1000 and int(match[3])>=600:
            candidates.append(match[1])
    if len(candidates)!=1:
        raise ValueError('Expected one main RViz window; found '+repr(candidates))
    return candidates[0]
