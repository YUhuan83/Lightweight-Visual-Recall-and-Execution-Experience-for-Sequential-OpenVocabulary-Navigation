"""Synthetic normal/fault integration checks; excludes real navigation."""
import unittest
from offline_goal_chain_replay import run


class GoalChainReplayTests(unittest.TestCase):
    def test_physical_profile_confirmed_memory_normal_chain(self):
        result = run('ackermann', False)
        self.assertEqual([s['query'] for s in result['stages']],
                         ['gas station', 'green residential house', 'gas station'])
        self.assertTrue(all(s['found'] for s in result['stages']))
        self.assertTrue(all(s['actions'] > 0 for s in result['stages']))
        self.assertTrue(result['same_process_memory'])
        self.assertFalse(result['field_acceptance'])

    def test_point_profile_uses_unqueried_passive_memory_and_reconfirms(self):
        result = run('ideal_point', True)
        self.assertEqual([s['query'] for s in result['stages']],
                         ['green residential house', 'gas station'])
        self.assertEqual(result['stages'][-1]['memory_evidence'], 'passive_semantic_landmarks')
        self.assertTrue(result['same_process_memory'])
        self.assertTrue(result['passive_station_unqueried'])
        queries = [t['query'] for t in result['trace'] if t.get('query')]
        self.assertEqual(queries[0], 'green residential house')
        self.assertFalse(result['field_acceptance'])

    def test_extension_timeout_recovers_physical_normal_chain(self):
        result = run('ackermann', False, 'extension_timeout')
        self.assertTrue(result['recovered'])
        self.assertTrue(result['completed'])
        self.assertTrue(all(s['found'] for s in result['stages']))
        self.assertEqual(result['stages'][-1]['query'], 'gas station')
        self.assertFalse(result['field_acceptance'])

    def test_extension_timeout_recovers_passive_point_chain(self):
        result = run('ideal_point', True, 'extension_timeout')
        self.assertTrue(result['recovered'])
        self.assertTrue(result['completed'])
        self.assertTrue(all(s['found'] for s in result['stages']))
        self.assertEqual(result['stages'][-1]['memory_evidence'], 'passive_semantic_landmarks')
        self.assertFalse(result['field_acceptance'])


if __name__ == '__main__':
    unittest.main()
