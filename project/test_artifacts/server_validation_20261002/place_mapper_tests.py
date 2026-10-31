from pathlib import Path
p=Path(r'C:\Users\Yu\Desktop\拟真\wildos_port\repro_check\src\nebula2-wildos\graphnav_mapper\test\test_mapper.cpp')
s=p.read_text(encoding='utf-8')
a=s.index('TEST_F(MapperTest, PhysicalEvidenceAgeAndTraversalSurviveGraphPublication)')
b=s.index('#include "graphnav_mapper/mapper.hpp"',a)
tests=s[a:b]
p.write_text(s[:a]+s[b:]+'\n'+tests,encoding='utf-8')
print('moved mapper regression tests after fixture definition')

