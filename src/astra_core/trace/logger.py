"""Structured trace log (R10): job id, step/skill, target, plan success/
failure, execution result and recovery outcome, one JSON object per line so
it is greppable and machine-parseable. Both perception outcomes (accept and
reject) are recorded here - the acceptance evidence for the assessment."""
from __future__ import annotations

import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class TraceLogger:
    job_id: str
    out_path: Path | None = None
    _records: list[dict[str, Any]] = field(default_factory=list, init=False)

    def log(self, **fields: Any) -> dict[str, Any]:
        record = {"ts": time.time(), "job_id": self.job_id, **fields}
        self._records.append(record)
        line = json.dumps(record, default=str)
        print(line, file=sys.stderr)
        if self.out_path is not None:
            self.out_path.parent.mkdir(parents=True, exist_ok=True)
            with self.out_path.open("a") as f:
                f.write(line + "\n")
        return record

    @property
    def records(self) -> list[dict[str, Any]]:
        return list(self._records)
