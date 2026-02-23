#!/usr/bin/env python3
"""Repository self-check for non-ROS users.

Checks:
1) Python version
2) Required Python packages
3) Required project files
4) Minimal headless runtime commands
"""

from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def print_result(ok: bool, title: str, detail: str = "") -> None:
    status = "PASS" if ok else "FAIL"
    if detail:
        print(f"[{status}] {title}: {detail}")
    else:
        print(f"[{status}] {title}")


def check_python(min_major: int = 3, min_minor: int = 10) -> bool:
    v = sys.version_info
    ok = (v.major, v.minor) >= (min_major, min_minor)
    print_result(ok, "Python version", f"{v.major}.{v.minor}.{v.micro} (need >= {min_major}.{min_minor})")
    return ok


def check_imports(modules: list[str]) -> bool:
    all_ok = True
    for mod in modules:
        try:
            importlib.import_module(mod)
            print_result(True, f"Import {mod}")
        except Exception as exc:
            all_ok = False
            print_result(False, f"Import {mod}", str(exc))
    return all_ok


def check_paths(paths: list[Path]) -> bool:
    all_ok = True
    for p in paths:
        if p.exists():
            print_result(True, f"Path exists", str(p))
        else:
            all_ok = False
            print_result(False, f"Path exists", str(p))
    return all_ok


def run_cmd(name: str, cmd: list[str], timeout_sec: float) -> bool:
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(ROOT),
            text=True,
            capture_output=True,
            timeout=timeout_sec,
            check=False,
        )
    except Exception as exc:
        print_result(False, name, str(exc))
        return False

    if proc.returncode == 0:
        print_result(True, name, "exit=0")
        return True

    tail = "\n".join((proc.stdout + "\n" + proc.stderr).strip().splitlines()[-10:])
    print_result(False, name, f"exit={proc.returncode}\n{tail}")
    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run project self-check")
    parser.add_argument(
        "--urdf",
        default="external/linkerhand-urdf/o6/right/linkerhand_o6_right.urdf",
        help="URDF path used by runtime checks",
    )
    parser.add_argument(
        "--poses",
        default="config/poses.yaml",
        help="Pose YAML path used by runtime checks",
    )
    parser.add_argument(
        "--skip-runtime",
        action="store_true",
        help="Only run static checks (python/import/path), skip command execution",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    urdf = Path(args.urdf)
    poses = Path(args.poses)

    print("== move_hand self-check ==")
    ok_python = check_python()
    ok_imports = check_imports(["pybullet", "numpy", "yaml"])
    ok_paths = check_paths([ROOT / urdf, ROOT / poses, ROOT / "sim" / "sim_load.py"])

    ok_runtime = True
    if not args.skip_runtime:
        if not (ok_imports and ok_paths):
            print_result(False, "Runtime checks", "skipped because prerequisite checks failed")
            ok_runtime = False
        else:
            py = sys.executable
            ok_runtime = all(
                [
                    run_cmd(
                        "sim_load headless",
                        [py, "sim/sim_load.py", "--urdf", str(urdf), "--headless", "--duration", "0.2"],
                        timeout_sec=30,
                    ),
                    run_cmd(
                        "cli_control middle",
                        [
                            py,
                            "tools/cli_control.py",
                            "--urdf",
                            str(urdf),
                            "--poses",
                            str(poses),
                            "--headless",
                            "pose",
                            "middle",
                        ],
                        timeout_sec=30,
                    ),
                    run_cmd(
                        "middle-home regression",
                        [
                            py,
                            "sim/test_middle_home_cycle.py",
                            "--urdf",
                            str(urdf),
                            "--poses",
                            str(poses),
                            "--cycles",
                            "2",
                            "--transition-steps",
                            "10",
                            "--sim-steps-per-waypoint",
                            "3",
                            "--settle-steps",
                            "5",
                        ],
                        timeout_sec=60,
                    ),
                ]
            )

    all_ok = ok_python and ok_imports and ok_paths and ok_runtime
    print()
    if all_ok:
        print("SELF_CHECK PASS")
        return 0

    print("SELF_CHECK FAIL")
    print("Hint: activate your environment and run: python -m pip install -r requirements.txt")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
