"""R11 test 5 (part A) / R2: source-scan guard - no literal part/joint/
obstacle id or recipe coordinate may appear under src/. This is what makes
"same code, different recipe" a checked fact rather than an assertion."""
from pathlib import Path

FORBIDDEN_IDS = ["member_A", "member_B", '"J1"', "'J1'", "fixture", "keepout"]

SRC_ROOT = Path("src")


def test_no_forbidden_literal_ids_in_source():
    offenders = []
    for path in SRC_ROOT.rglob("*.py"):
        text = path.read_text()
        for token in FORBIDDEN_IDS:
            if token in text:
                offenders.append((str(path), token))
    assert offenders == [], f"hard-coded product identifiers found: {offenders}"
