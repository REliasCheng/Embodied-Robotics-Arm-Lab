"""Versioned, bounded synthetic trace I/O."""

import json
from pathlib import Path

from offline_control.protocol import Sample

MAX_BYTES = 64 * 1024
MAX_SAMPLES = 100


class TraceError(ValueError):
    """Rejected trace file."""


def deterministic_samples(names: tuple[str, ...]) -> list[Sample]:
    return [Sample(tick=i, sequence=i, source="synthetic-v1",
                   channels={name: float(i + position) for position, name in enumerate(names)})
            for i in range(1, 4)]


def save_trace(path: Path, samples: list[Sample]) -> None:
    if len(samples) > MAX_SAMPLES:
        raise TraceError("too many samples")
    data = {"version": 1, "kind": "synthetic-offline",
            "samples": [{"tick": s.tick, "sequence": s.sequence, "source": s.source,
                         "channels": s.channels} for s in samples]}
    encoded = json.dumps(data, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(encoded) > MAX_BYTES:
        raise TraceError("trace too large")
    path.write_bytes(encoded)


def load_trace(path: Path) -> list[Sample]:
    if path.stat().st_size > MAX_BYTES:
        raise TraceError("trace too large")

    def unique_pairs(pairs: list[tuple[str, object]]) -> dict:
        result = {}
        for key, value in pairs:
            if key in result:
                raise TraceError(f"duplicate key: {key}")
            result[key] = value
        return result

    def bad_constant(value: str) -> None:
        raise TraceError(f"invalid constant: {value}")

    try:
        data = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=unique_pairs,
                          parse_constant=bad_constant)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise TraceError("invalid trace JSON") from exc
    if (not isinstance(data, dict) or set(data) != {"version", "kind", "samples"}
            or type(data["version"]) is not int or data["version"] != 1
            or data["kind"] != "synthetic-offline"
            or not isinstance(data["samples"], list)
            or len(data["samples"]) > MAX_SAMPLES):
        raise TraceError("invalid trace envelope")
    result = []
    for item in data["samples"]:
        if not isinstance(item, dict) or set(item) != {"tick", "sequence", "source", "channels"}:
            raise TraceError("invalid sample shape")
        if not isinstance(item["channels"], dict):
            raise TraceError("invalid channels")
        result.append(Sample(**item))
    return result
