import json
import sqlite3
from types import SimpleNamespace
import unittest
from model_identity_helpers import database_snapshot, all_inprocess

class ModelIdentityRows(unittest.TestCase):
    def test_actual_row_factory_counterexample_and_safe_snapshot(self):
        with sqlite3.connect(':memory:') as connection:
            connection.row_factory=sqlite3.Row
            with self.assertRaises(TypeError):json.dumps(connection.execute('PRAGMA database_list').fetchall())
            data=database_snapshot({'store':SimpleNamespace(_db=connection)})
            self.assertEqual(json.loads(json.dumps(data)),{'store':[[0,'main','']]})
            self.assertTrue(all_inprocess(data))

    def test_tuple_factory_is_also_json_serializable(self):
        with sqlite3.connect(':memory:') as connection:
            data=database_snapshot({'store':SimpleNamespace(_db=connection)})
            self.assertTrue(all_inprocess(json.loads(json.dumps(data))))

    def test_missing_or_persistent_database_is_not_declared_inprocess(self):
        self.assertFalse(all_inprocess(database_snapshot({'store':None})))
        self.assertFalse(all_inprocess({}))
        self.assertFalse(all_inprocess({'store':[[0,'main','persistent.sqlite']]}))

if __name__=='__main__':unittest.main()
