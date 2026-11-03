"""Verify the installed typing artifacts and the complete documented recipes."""

import ast
from contextlib import redirect_stdout
import io
from pathlib import Path
import runpy
import unittest

import dynibo


class DocumentationTests(unittest.TestCase):
    def test_wheel_contains_complete_public_type_information(self):
        package = Path(dynibo.__file__).parent
        self.assertTrue((package / "py.typed").is_file())
        public = ast.parse((package / "__init__.pyi").read_text(encoding="utf-8"))
        exports = {
            item.asname
            for node in public.body
            if isinstance(node, ast.ImportFrom)
            for item in node.names
        }
        self.assertEqual(exports, set(dynibo.__all__))
        native = ast.parse((package / "_dynibo.pyi").read_text(encoding="utf-8"))
        classes = {
            node.name: node for node in native.body if isinstance(node, ast.ClassDef)
        }
        self.assertEqual(set(classes), set(dynibo.__all__))
        for name, stub in classes.items():
            runtime = getattr(dynibo, name)
            documented = {
                member.name
                for member in stub.body
                if isinstance(member, ast.FunctionDef)
            }
            public_members = {
                name for name in vars(runtime) if not name.startswith("_")
            }
            with self.subTest(type=name):
                self.assertLessEqual(public_members, documented)

    def test_standalone_task_recipes(self):
        recipes = Path(__file__).resolve().parents[3] / "examples/python/recipes"
        scripts = sorted(recipes.glob("*.py"))
        self.assertEqual(len(scripts), 6)
        for script in scripts:
            with self.subTest(recipe=script.name), redirect_stdout(io.StringIO()):
                runpy.run_path(str(script), run_name="__main__")
