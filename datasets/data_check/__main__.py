"""python -m datasets.data_check <subcommand> ..."""

from __future__ import annotations

import sys
from pathlib import Path

# Local vendored deps (e.g. joblib) when conda env lacks write permission.
_VENDOR = Path(__file__).resolve().parent / "_vendor"
if _VENDOR.is_dir() and str(_VENDOR) not in sys.path:
    sys.path.insert(0, str(_VENDOR))


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in {"-h", "--help"}:
        print(
            "Usage:\n"
            "  python -m datasets.data_check convert ...\n"
            "  python -m datasets.data_check analyze ...\n"
            "  python -m datasets.data_check isaac ...\n"
            "  python -m datasets.data_check pipeline ...\n"
        )
        return 0
    cmd, rest = argv[0], argv[1:]
    if cmd == "convert":
        from .convert_to_mode2 import main as m
    elif cmd == "analyze":
        from .analyze_joint_stability import main as m
    elif cmd == "isaac":
        from .isaac_batch_validate import main as m
    elif cmd in {"pipeline", "run"}:
        from .run_pipeline import main as m
    else:
        print(f"Unknown subcommand: {cmd}", file=sys.stderr)
        return 2
    return int(m(rest))


if __name__ == "__main__":
    raise SystemExit(main())
