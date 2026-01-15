#!/usr/bin/env python3
import argparse
from pathlib import Path
from typing import List, Tuple

def parse_table(lines: List[str]) -> Tuple[int, int, List[List[str]]]:
    start = end = -1
    rows: List[List[str]] = []
    for i, line in enumerate(lines):
        if line.lstrip().startswith("|"):
            if start == -1:
                start = i
            end = i
            # split on |, drop first/last empties after strip
            cells = [c.strip() for c in line.strip().split("|")[1:-1]]
            rows.append(cells)
        elif start != -1:
            break
    return start, end, rows

def format_table(rows: List[List[str]]) -> List[str]:
    if not rows:
        return []
    col_count = max(len(r) for r in rows)
    # pad missing cells
    for r in rows:
        r += [""] * (col_count - len(r))
    widths = [max(len(r[c]) for r in rows) for c in range(col_count)]
    formatted = []
    for idx, r in enumerate(rows):
        line = "| " + " | ".join(r[c].ljust(widths[c]) for c in range(col_count)) + " |"
        formatted.append(line)
        # keep the separator row as-is (second line)
        if idx == 0:
            sep = "| " + " | ".join("-" * widths[c] for c in range(col_count)) + " |"
            formatted.append(sep)
    return formatted

def main():
    ap = argparse.ArgumentParser(description="Normalize Markdown table widths.")
    ap.add_argument("input", type=Path, help="Input markdown file")
    ap.add_argument("-o", "--output", type=Path, help="Output file (default: overwrite input)")
    args = ap.parse_args()

    text = args.input.read_text(encoding="utf-8").splitlines()
    start, end, rows = parse_table(text)
    if start == -1:
        raise SystemExit("No table found.")
    formatted = format_table(rows)
    output_lines = text[:start] + formatted + text[end + 1 :]
    target = args.output or args.input
    target.write_text("\n".join(output_lines) + "\n", encoding="utf-8")
    print(f"Formatted table written to {target}")

if __name__ == "__main__":
    main()