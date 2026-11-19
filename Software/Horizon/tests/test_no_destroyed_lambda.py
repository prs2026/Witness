"""``destroyed`` must never be connected to a lambda or closure.

When a widget tree is released by the cyclic GC, the collector clears every
function in the garbage and the C++ destructors then emit ``destroyed`` into the
half-cleared functions: an access violation inside ``gc.collect``. Connect a
callable object (``qt_widgets._OnDestroyed``) instead.
"""

import ast
import pathlib
import unittest

ROOTS = (pathlib.Path(__file__).resolve().parents[1],
         pathlib.Path(__file__).resolve().parents[2] / "witnessOS")


class NoDestroyedLambda(unittest.TestCase):
    def test_destroyed_slots_are_callable_objects(self):
        offenders = []
        for root in ROOTS:
            for path in root.rglob("*.py"):
                if any(part in (".venv", "__pycache__", "tests") for part in path.parts):
                    continue
                try:
                    tree = ast.parse(path.read_text(encoding="utf-8"))
                except (SyntaxError, UnicodeDecodeError):
                    continue
                for node in ast.walk(tree):
                    if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                            and node.func.attr == "connect"
                            and isinstance(node.func.value, ast.Attribute)
                            and node.func.value.attr == "destroyed"
                            and node.args
                            and isinstance(node.args[0], (ast.Lambda, ast.Name, ast.FunctionDef))):
                        offenders.append(f"{path}:{node.lineno}")
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
