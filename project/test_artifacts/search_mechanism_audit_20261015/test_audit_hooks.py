import importlib.util,unittest
from pathlib import Path
p=Path(__file__).with_name('audit_vision.py');s=importlib.util.spec_from_file_location('audit',p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
class Tests(unittest.TestCase):
 def test_observer_preserves_return_and_input_identity(self):
  value=[];seen=[];f=m.observed(lambda x:x,lambda a,k,r:seen.append((a[0],r)),[])
  self.assertIs(f(value),value);self.assertIs(seen[0][0],value);self.assertIs(seen[0][1],value)
 def test_audit_error_cannot_change_result(self):
  errors=[]
  def bad(*a):raise RuntimeError('observer failure')
  self.assertEqual(m.observed(lambda:7,bad,errors)(),7);self.assertEqual(len(errors),1)
 def test_original_exception_is_preserved(self):
  seen=[]
  def bad():raise ValueError('original')
  with self.assertRaisesRegex(ValueError,'original'):m.observed(bad,lambda *a:seen.append(1),[])()
  self.assertEqual(seen,[])
if __name__=='__main__':unittest.main()
