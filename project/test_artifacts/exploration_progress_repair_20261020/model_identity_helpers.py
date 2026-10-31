"""JSON-safe metadata from the actual SQLite row factory used by the stores."""
def database_snapshot(stores):
    return {name: [list(row) for row in store._db.execute('PRAGMA database_list').fetchall()]
            if store is not None else [] for name,store in stores.items()}

def all_inprocess(databases):
    return bool(databases) and all(rows and all(not row[2] for row in rows)
                                   for rows in databases.values())
