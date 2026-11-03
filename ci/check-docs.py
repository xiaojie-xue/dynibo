"""Check built reference content so an empty API page cannot pass CI."""

import ast
from html.parser import HTMLParser
from pathlib import Path
import sys


class Page(HTMLParser):
    def __init__(self, path: Path):
        super().__init__()
        self.ids: set[str] = set()
        self.text: list[str] = []
        self.feed(path.read_text(encoding="utf-8"))

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if "id" in attributes:
            self.ids.add(attributes["id"])

    def handle_data(self, data):
        self.text.append(data)


def main():
    root = Path(__file__).resolve().parents[1]
    site = Path(sys.argv[1]) if len(sys.argv) > 1 else root / "site"
    stub = ast.parse((root / "bindings/python/dynibo/_dynibo.pyi").read_text())
    expected = set()
    for node in stub.body:
        if isinstance(node, ast.ClassDef):
            expected.add(f"dynibo.{node.name}")
            expected.update(
                f"dynibo.{node.name}.{member.name}"
                for member in node.body
                if isinstance(member, ast.FunctionDef)
                and not member.name.startswith("_")
            )
    for language in ("", "zh"):
        page = Page(site / language / "reference/python/index.html")
        missing = expected - page.ids
        if missing:
            raise SystemExit(f"Python reference missing API entries: {sorted(missing)}")
        text = "".join(page.text)
        for contract in ("out", "NDArray", "ValueError", "column-major"):
            if contract not in text:
                raise SystemExit(f"Python reference missing contract: {contract}")
        for script in sorted((root / "examples/python/recipes").glob("*.py")):
            path = script.stem.replace("_", "-")
            recipe = Page(site / language / "recipes" / path / "index.html")
            if "TemporaryDirectory" not in "".join(recipe.text):
                raise SystemExit(f"Recipe code was not expanded: {path}")
    print(
        f"Verified {len(expected)} Python API entries and six recipe pages in both languages."
    )


if __name__ == "__main__":
    main()
