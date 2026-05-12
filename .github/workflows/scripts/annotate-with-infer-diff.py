#!/usr/bin/env python3
import json
import sys


def main(path):
    with open(path) as f:
        report = json.load(f)

    for bug in report:
        level = "error" if bug.get("severity") == "ERROR" else "warning"
        file = bug.get("file")
        line = bug.get("line")
        col = bug.get("column")
        bug_type = bug.get("bug_type_hum", bug.get("bug_type"))

        if None in (file, line, col, bug_type):
            continue  # Skip incomplete entries

        message = f"{bug_type}: {bug.get('qualifier', '')}"
        suggestion = bug.get("suggestion")
        if suggestion:
            message += f"\nSuggestion: {suggestion}"
        # Escape colons and percent signs as required by GitHub Actions
        message = (
            message.replace("%", "%25")
            .replace("\r", "%0D")
            .replace("\n", "%0A")
            .replace(":", "\\:")
        )
        print(f"::{level} file={file},line={line},col={col}::{message}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} report.json", file=sys.stderr)
        sys.exit(1)
    main(sys.argv[1])
