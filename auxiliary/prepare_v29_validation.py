from pathlib import Path
base=Path(r'C:\Users\Yu\Desktop\Beamng')
s=(base/'validate_v28_remote.py').read_text().replace('v28_intent_handoff','v29_range_consistency')
s=s.replace('import test_fresh_bearing','import test_fresh_bearing\nimport test_depth_reference\nfrom sensor_msgs_py import point_cloud2\nfrom builtin_interfaces.msg import Time\nfrom std_msgs.msg import Header')
where="suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(ImportedCallbackTests)])"
block='''class ImportedDepthTests(test_depth_reference.DepthReferenceTests):
    def set_input(self,t,points):
        super().set_input(t,points)
        self.msg.header.stamp=Time(sec=t//10**9,nanosec=t%10**9)
        self.cloud=point_cloud2.create_cloud_xyz32(Header(stamp=self.msg.header.stamp,frame_id='world'),points.tolist())
    def setUp(self):
        super().setUp()
        actual=ObjectMaskTriangulator.__new__(ObjectMaskTriangulator)
        actual.__dict__.update(self.n.__dict__)
        self.n=actual

suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(ImportedCallbackTests),
                        unittest.defaultTestLoader.loadTestsFromTestCase(ImportedDepthTests)])'''
assert where in s;s=s.replace(where,block)
s=s.replace("('test_localization_geometry',","('test_range_consistency','test_localization_geometry',")
(base/'validate_v29_remote.py').write_text(s)
s=(base/'run_v28_field.py').read_text().replace('v28_intent_handoff','v29_range_consistency')
(base/'run_v29_field.py').write_text(s)
