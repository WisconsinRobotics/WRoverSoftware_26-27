#!/usr/bin/env python3

import ast
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

EXPECTED = {
    "maintainer": "Wisconsin Robotics",
    "maintainer_email": "wisconsinrobotics@gmail.com",
    "license": "MIT",
}


def pkg_xml_fields(path: Path) -> dict:
    """Read the first maintainer, email, and license from an XML file."""
    root = ET.parse(path).getroot()
    maint = root.find("maintainer")
    return {
        "maintainer": (root.findtext("maintainer") or "").strip(),
        "maintainer_email": maint.get("email", "") if maint is not None else "",
        "license": (root.findtext("license") or "").strip(),
    }


def setup_py_fields(path: Path) -> dict:
    """Read string keyword args to find maintainer, email, and license data."""
    for node in ast.walk(ast.parse(path.read_text())):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "setup":
            return {
                kw.arg: kw.value.value
                for kw in node.keywords
                if isinstance(kw.value, ast.Constant)
                and isinstance(kw.value.value, str)
            }
    return dict()


def check_package(pkg_dir: Path) -> list[str]:
    errors = []
    for path, read_fields in (
        (pkg_dir / "package.xml", pkg_xml_fields),
        (pkg_dir / "setup.py", setup_py_fields),
    ):
        if not path.exists():
            continue
        fields = read_fields(path)
        for key, val in EXPECTED.items():
            if fields.get(key) != val:
                errors.append(f"{path}: {key}={fields.get(key)!r} != {val!r}")

    return errors


def main(files: list[str]) -> int:
    errors = []
    for d in sorted({Path(f).parent for f in files}):
        errors += check_package(d)
    for e in errors:
        print(e)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
