import argparse
from collections import defaultdict
import json
from pathlib import Path
from typing import Dict
from typing import List


def load_bugs(filepath: Path) -> List[Dict]:
    with open(filepath, "r") as f:
        return json.load(f)


def count_by_type(bugs: List[Dict]) -> Dict[str, int]:
    counts = defaultdict(int)
    for bug in bugs:
        counts[bug["bug_type_hum"]] += 1
    return dict(counts)


def create_markdown_table(
    pre_counts: Dict[str, int],
    fixed_counts: Dict[str, int],
    introduced_counts: Dict[str, int],
) -> str:
    # Get all unique bug types
    all_bug_types = set()
    all_bug_types.update(pre_counts.keys())
    all_bug_types.update(fixed_counts.keys())
    all_bug_types.update(introduced_counts.keys())

    lines = []
    lines.append("| Bug Type | Pre-existing | Fixed | Introduced |")
    lines.append("|----------|-------------:|------:|-----------:|")

    # Add rows
    for bug_type in sorted(all_bug_types):
        pre = pre_counts.get(bug_type, 0)
        fix = fixed_counts.get(bug_type, 0)
        intro = introduced_counts.get(bug_type, 0)
        lines.append(f"| {bug_type} | {pre} | {fix} | {intro} |")

    return "\n".join(lines)


def summarize_diff(
    fixed_counts: Dict[str, int],
    introduced_counts: Dict[str, int],
) -> str:
    if sum(introduced_counts.values()) == 0 and sum(fixed_counts.values()) > 0:
        return f"Fixed {sum(fixed_counts.values())} bugs without introducing new ones! 🎉"
    elif sum(introduced_counts.values()) == 0:
        return "No new bugs introduced ✅"
    elif sum(fixed_counts.values()) > 0:
        return f"Fixed {sum(fixed_counts.values())} bugs but introduced {sum(introduced_counts.values())} new ones ⚠️"
    else:
        return f"Introduced {sum(introduced_counts.values())} new bugs without fixing any existing ones ❌"


def main():
    parser = argparse.ArgumentParser(
        description="Summarize Infer differential analysis results as markdown"
    )
    parser.add_argument(
        "differential_analysis_path",
        type=str,
        help="Path to the `differential` subdirectory of an infer-out directory",
    )
    parser.add_argument(
        "artifact_url",
        type=str,
        help="URL of the uploaded Infer report",
    )
    parser.add_argument(
        "output_path",
        type=str,
        help="Path to save the markdown table",
    )
    args = parser.parse_args()
    base_path = Path(args.differential_analysis_path)

    preexisting = load_bugs(base_path / "preexisting.json")
    fixed = load_bugs(base_path / "fixed.json")
    introduced = load_bugs(base_path / "introduced.json")

    pre_counts = count_by_type(preexisting)
    fixed_counts = count_by_type(fixed)
    introduced_counts = count_by_type(introduced)

    header = f"## [FB Infer]({args.artifact_url}) Report"
    comment = summarize_diff(fixed_counts, introduced_counts)
    table = create_markdown_table(pre_counts, fixed_counts, introduced_counts)
    artifact_comment = f"[Download the full Infer report]({args.artifact_url})."

    markdown = (
        f"{header}\n\n"
        f"{comment}\n\n"
        f"<details><summary>Additional details</summary>\n\n"
        f"{table}\n\n"
        f"</details>\n\n"
        f"{artifact_comment}"
    )

    # Write to file
    with open(args.output_path, "w") as f:
        f.write(markdown)


if __name__ == "__main__":
    main()
