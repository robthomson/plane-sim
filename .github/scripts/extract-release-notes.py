#!/usr/bin/env python3
import sys


def extract_release_notes(version, path):
    header = f"# {version}"
    notes = []
    found = False

    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip() == header:
                found = True
                notes.append(line)
                continue

            if found and line.startswith("# "):
                break

            if found:
                notes.append(line)

    if not found:
        return None

    return "".join(notes).strip() + "\n"


def main():
    if len(sys.argv) != 3:
        print("usage: extract-release-notes.py <version> <Releases.md>", file=sys.stderr)
        return 2

    version = sys.argv[1].strip()
    path = sys.argv[2]
    notes = extract_release_notes(version, path)
    if notes:
        print(notes)
        return 0

    print(f"# {version}\n\n- Release notes not found in {path}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
