"""Generate host-only validation artifacts in this owned directory, no ROS needed."""
import io
import json
import platform
import unittest

import preflight
import test_preflight


if __name__ == '__main__':
    output = io.StringIO()
    result = unittest.TextTestRunner(stream=output, verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromModule(test_preflight))
    report = preflight.preflight(preflight.DEFAULT_SOURCE)
    artifacts = {
        'host_preflight.json': report,
        'dependency_graph.json': preflight.inventory(preflight.DEFAULT_SOURCE),
        'validation.json': {'platform': platform.platform(), 'tests_run': result.testsRun,
                            'failures': len(result.failures), 'errors': len(result.errors),
                            'skipped': len(result.skipped), 'output': output.getvalue(),
                            'linux_build_performed': False},
    }
    for name, data in artifacts.items():
        (preflight.HERE / name).write_text(json.dumps(data, indent=2), encoding='utf-8')
    print(output.getvalue())
    print('Host preflight:', report['status'])
    print('Linux/ROS build performed: False')
    raise SystemExit(0 if result.wasSuccessful() else 1)
