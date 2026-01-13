#!/usr/bin/env python3
"""Preprocess documentation for Zensical by expanding json_to_markdown macros.

This script prepares a Zensical-compatible docs tree by:
1. Copying source docs to an output directory
2. Merging mkdoxy-generated API reference markdown
3. Expanding {{ json_to_markdown(...) }} macros into GitHub-Flavored Markdown tables
4. Generating a derived Zensical config with updated paths
"""

import argparse
import ast
import json
import os
import re
import shutil
import sys
from pathlib import Path

from tabulate import tabulate


ROOT = Path(__file__).resolve().parents[1]


_MARKDOWN_LINK_RE = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")


def _md_links_to_html(text: str) -> str:
    """Convert markdown links with absolute paths to HTML anchor tags."""
    def repl(match: re.Match) -> str:
        label = match.group(1)
        href = match.group(2)
        if not href.startswith("/"):
            return match.group(0)
        return f'<a href="{href}">{label}</a>'

    return _MARKDOWN_LINK_RE.sub(repl, text)


def format_param_type(param_type: str) -> str:
    """Convert JSON schema type to display type (e.g., 'number' -> 'float')."""
    if param_type == "number":
        return "float"
    return param_type


def ensure_word_breaks(text: str) -> str:
    """Insert HTML word break hints after slashes for better table rendering."""
    if not isinstance(text, str):
        return text
    return text.replace("/", "/<wbr>")


def format_param_range(param: dict) -> str:
    """Format JSON schema constraints (min, max, enum) as a human-readable string."""
    list_of_range = []
    if "enum" in param:
        list_of_range.append(", ".join(map(str, param["enum"])))
    if "minimum" in param:
        list_of_range.append("≥ " + str(param["minimum"]))
    if "exclusiveMinimum" in param:
        list_of_range.append("> " + str(param["exclusiveMinimum"]))
    if "maximum" in param:
        list_of_range.append("≤ " + str(param["maximum"]))
    if "exclusiveMaximum" in param:
        list_of_range.append("< " + str(param["exclusiveMaximum"]))
    if "exclusive" in param:
        list_of_range.append("≠ " + str(param["exclusive"]))

    if not list_of_range:
        return "N/A"

    return "<br/>".join(map(str, list_of_range))


def get_json_path(json_data: dict, json_path: list) -> dict:
    """Traverse a nested dict/list structure using a path of keys/indices."""
    for elem in json_path:
        if isinstance(elem, int):
            json_data = list(json_data.values())[elem]
        else:
            json_data = json_data[elem]
    return json_data


def extract_parameter_info(
    parameters: dict,
    namespace: str = "",
    file_directory: str = "",
    include_refs: bool = False,
) -> list[dict]:
    """Recursively extract parameter metadata from a JSON schema properties dict."""
    params = []
    for k, v in parameters.items():
        if not isinstance(v, dict):
            continue

        if "$ref" in v:
            if not include_refs:
                continue

            ref: str = v["$ref"]
            ref_path, ref_json_path = ref.split("#")
            ref_path = os.path.join(file_directory, ref_path)
            ref_json_path = ref_json_path.split("/")
            if ref_json_path and ref_json_path[0] == "":
                ref_json_path = ref_json_path[1:]

            with open(ref_path, encoding="utf-8") as f:
                data = json.load(f)
            param = get_json_path(data, ref_json_path)

            param.update({kk: vv for kk, vv in v.items() if kk != "$ref"})

            extracted_from_ref = extract_parameter_info(
                {k: param}, namespace, os.path.split(ref_path)[0], include_refs
            )
            params.extend(extracted_from_ref)
            continue

        if v.get("type") != "object":
            description = v.get("description", "")
            if isinstance(description, str):
                description = _md_links_to_html(description)
            param = {
                "Name": namespace + k,
                "Type": format_param_type(v.get("type", "")),
                "Description": description,
                "Default": ensure_word_breaks(v.get("default", "")),
                "Range": format_param_range(v),
            }
            params.append(param)
            continue

        props = v.get("properties")
        if isinstance(props, dict):
            params.extend(extract_parameter_info(props, k + ".", file_directory, include_refs))

    return params


def to_gfm_table(rows: list[dict]) -> str:
    """Convert a list of dicts to a GitHub-Flavored Markdown table."""
    return tabulate(rows, headers="keys", tablefmt="github")


def json_to_markdown(
    json_schema_file_path: str,
    json_path: list | None = None,
    include_refs: bool = True,
) -> str:
    """Convert a JSON schema file to a GitHub-Flavored Markdown parameter table.

    Args:
        json_schema_file_path: Path to the JSON schema file.
        json_path: Path within the JSON to the properties dict.
        include_refs: Whether to follow $ref references.

    Returns:
        A GFM table string listing parameter names, types, defaults, and ranges.
    """
    if json_path is None:
        json_path = ["definitions", 0, "properties"]

    with open(json_schema_file_path, encoding="utf-8") as f:
        data = json.load(f)

    params = get_json_path(data, json_path)
    param_info = extract_parameter_info(
        params,
        file_directory=os.path.split(json_schema_file_path)[0],
        include_refs=include_refs,
    )
    return to_gfm_table(param_info)


def _parse_python_call_args(args_src: str) -> tuple[list, dict]:
    """Parse a Python function call's arguments from source text."""
    expr = ast.parse(f"f({args_src})", mode="eval").body
    if not isinstance(expr, ast.Call):
        raise ValueError("unexpected AST (not a call)")

    def lit(node):
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.List):
            return [lit(e) for e in node.elts]
        if isinstance(node, ast.Tuple):
            return tuple(lit(e) for e in node.elts)
        if isinstance(node, ast.Dict):
            return {lit(k): lit(v) for k, v in zip(node.keys, node.values)}
        raise ValueError(f"unsupported literal in macro args: {type(node).__name__}")

    positional = [lit(a) for a in expr.args]
    keywords = {}
    for kw in expr.keywords:
        if kw.arg is None:
            raise ValueError("**kwargs not supported in macro args")
        keywords[kw.arg] = lit(kw.value)
    return positional, keywords


_MACRO_RE = re.compile(
    r"\{\{\s*json_to_markdown\s*\((?P<args>.*?)\)\s*\}\}",
    flags=re.DOTALL,
)


def render_macros_in_markdown_file(path: Path) -> bool:
    """Expand json_to_markdown macros in a markdown file in-place.

    Returns True if any macros were expanded, False otherwise.
    """
    text = path.read_text(encoding="utf-8")
    if "{{" not in text or "json_to_markdown" not in text:
        return False

    def repl(match: re.Match) -> str:
        args_src = match.group("args")
        pos, kw = _parse_python_call_args(args_src)
        return str(json_to_markdown(*pos, **kw))

    new_text, count = _MACRO_RE.subn(repl, text)
    if count == 0:
        return False

    path.write_text(new_text, encoding="utf-8")
    return True


def write_derived_zensical_config(
    src: Path,
    dst: Path,
    docs_dir: str,
    site_dir: str,
    site_url: str | None,
) -> None:
    """Generate a derived Zensical config with updated docs_dir, site_dir, and site_url."""
    lines = src.read_text(encoding="utf-8").splitlines(keepends=True)
    found_docs_dir = False
    found_site_dir = False
    found_site_url = False
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("docs_dir"):
            lines[i] = f'docs_dir = "{docs_dir}"\n'
            found_docs_dir = True
        if stripped.startswith("site_dir"):
            lines[i] = f'site_dir = "{site_dir}"\n'
            found_site_dir = True
        if site_url and stripped.startswith("site_url"):
            lines[i] = f'site_url = "{site_url}"\n'
            found_site_url = True
    if not found_docs_dir:
        lines.insert(0, f'docs_dir = "{docs_dir}"\n')
    if not found_site_dir:
        lines.insert(0, f'site_dir = "{site_dir}"\n')
    if site_url and not found_site_url:
        lines.insert(0, f'site_url = "{site_url}"\n')

    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text("".join(lines), encoding="utf-8")


def main() -> int:
    """CLI entrypoint: preprocess docs and generate derived Zensical config."""
    parser = argparse.ArgumentParser(
        description="Prepare a Zensical-compatible docs tree (expands json_to_markdown macros)."
    )
    parser.add_argument(
        "--docs-src",
        default=str(ROOT / "docs"),
        help="Source docs directory.",
    )
    parser.add_argument(
        "--mkdoxy-dir",
        default=str(ROOT / ".mkdoxy"),
        help="Directory containing mkdoxy-generated markdown (optional).",
    )
    parser.add_argument(
        "--out-dir",
        default=str(ROOT / ".zensical" / "docs"),
        help="Output docs directory (generated).",
    )
    parser.add_argument(
        "--config-src",
        default=str(ROOT / "zensical.toml"),
        help="Input Zensical config.",
    )
    parser.add_argument(
        "--config-out",
        default=str(ROOT / ".zensical.toml"),
        help="Output Zensical config (generated).",
    )
    parser.add_argument(
        "--site-dir",
        default="_site",
        help="site_dir to write into the generated config.",
    )
    parser.add_argument(
        "--site-url",
        default=os.environ.get("ZENSICAL_SITE_URL"),
        help="site_url to write into the generated config (optional).",
    )
    args = parser.parse_args()

    docs_src = Path(args.docs_src)
    mkdoxy_dir = Path(args.mkdoxy_dir)
    out_dir = Path(args.out_dir)
    config_src = Path(args.config_src)
    config_out = Path(args.config_out)

    if not docs_src.exists():
        print(f"docs source not found: {docs_src}", file=sys.stderr)
        return 2

    if out_dir.exists():
        shutil.rmtree(out_dir)
    shutil.copytree(docs_src, out_dir)

    if mkdoxy_dir.exists():
        # Only copy rendered markdown pages from mkdoxy output (skip doxygen XML, hashes, etc.).
        for md in mkdoxy_dir.rglob("*.md"):
            rel = md.relative_to(mkdoxy_dir)
            target = out_dir / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(md, target)

    rendered = 0
    for md in out_dir.rglob("*.md"):
        if render_macros_in_markdown_file(md):
            rendered += 1

    docs_dir_in_config = os.path.relpath(out_dir, start=ROOT)
    site_url = (args.site_url or "").strip() or None
    write_derived_zensical_config(config_src, config_out, docs_dir_in_config, args.site_dir, site_url)

    print(f"Prepared docs in {docs_dir_in_config} (rendered {rendered} file(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
