#!/usr/bin/env python3
"""
Auto-register a new planner method in system.launch.py.

Usage:
  python scripts/register_planner_method.py --method my_algo --entry my_planner.py
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


def _insert_once(text: str, anchor: str, payload: str, after: bool = True) -> str:
    if payload.strip() in text:
        return text
    idx = text.find(anchor)
    if idx < 0:
        raise RuntimeError(f"Anchor not found: {anchor!r}")
    insert_at = idx + len(anchor) if after else idx
    return text[:insert_at] + payload + text[insert_at:]


def _append_method_in_description(text: str, method: str) -> str:
    pat = re.compile(r'(description="Planner branch to run:\s*)([^"]+)(")')
    m = pat.search(text)
    if not m:
        raise RuntimeError("planner_method description not found")
    branches = [x.strip() for x in m.group(2).split("|")]
    if method not in branches:
        branches.append(method)
    new_desc = f'{m.group(1)}{" | ".join(branches)}{m.group(3)}'
    return text[: m.start()] + new_desc + text[m.end() :]


def register_method(launch_path: Path, method: str, entry: str, root_subdir: str) -> None:
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", method):
        raise ValueError("method must match [A-Za-z_][A-Za-z0-9_]*")
    if "/" in entry:
        raise ValueError("entry must be a filename, not a path")

    text = launch_path.read_text(encoding="utf-8")
    upper = method.upper()
    root_var = f"{method}_root"
    root_arg_var = f"{method}_root_arg"
    node_var = f"{method}_node"
    default_const = f"_DEFAULT_{upper}"

    const_block = (
        f'{default_const} = str(Path(__file__).resolve().parents[5] / "policy" / "{root_subdir}")\n'
    )
    text = _insert_once(
        text,
        '_DEFAULT_SUPER = str(Path(__file__).resolve().parents[5] / "policy" / "super")\n',
        const_block,
        after=True,
    )

    cfg_line = f"    {root_var} = LaunchConfiguration(\"{root_var}\")\n"
    text = _insert_once(
        text,
        '    super_root = LaunchConfiguration("super_root")\n',
        cfg_line,
        after=True,
    )

    arg_block = (
        f"    {root_arg_var} = DeclareLaunchArgument(\n"
        f"        \"{root_var}\",\n"
        f"        default_value={default_const},\n"
        f"        description=\"Absolute path to the policy/{root_subdir}/ folder.\",\n"
        f"    )\n"
    )
    text = _insert_once(text, "    super_root_arg = DeclareLaunchArgument(\n", arg_block, after=False)

    text = _append_method_in_description(text, method)

    node_block = (
        f"\n    {node_var} = ExecuteProcess(\n"
        f"        cmd=[\n"
        f"            \"bash\",\n"
        f"            \"-lc\",\n"
        f"            [\n"
        f"                \"source /opt/ros/${{ROS_DISTRO:-humble}}/setup.bash\",\n"
        f"                \" && if [ -f '\",\n"
        f"                _DEFAULT_CONTROLLER_SETUP,\n"
        f"                \"' ]; then source '\",\n"
        f"                _DEFAULT_CONTROLLER_SETUP,\n"
        f"                \"'; elif [ -f '\",\n"
        f"                _DEFAULT_CONTROLLER_SETUP_ALT,\n"
        f"                \"' ]; then source '\",\n"
        f"                _DEFAULT_CONTROLLER_SETUP_ALT,\n"
        f"                \"'; else echo '[system.launch] Missing controller setup.bash' >&2; exit 1; fi\",\n"
        f"                \" && python3 \",\n"
        f"                {root_var},\n"
        f"                \"/{entry}\",\n"
        f"                \" --ros-args\",\n"
        f"                \" -p odom_topic:=\",\n"
        f"                odom_topic,\n"
        f"                \" -p lidar_topic:=\",\n"
        f"                lidar_topic,\n"
        f"                \" -p ctrl_topic:=\",\n"
        f"                ctrl_topic,\n"
        f"            ],\n"
        f"        ],\n"
        f"        output=\"screen\",\n"
        f"        condition=IfCondition(PythonExpression([\"'\", planner_method, \"' == '{method}'\"])),\n"
        f"    )\n"
    )
    text = _insert_once(text, "    super_compat_node = ExecuteProcess(\n", node_block, after=False)

    text = _insert_once(text, "        super_root_arg,\n", f"        {root_arg_var},\n", after=True)
    text = _insert_once(text, "        super_compat_node,\n", f"        {node_var},\n", after=True)

    launch_path.write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", required=True, help="planner_method value, e.g. my_algo")
    parser.add_argument("--entry", required=True, help="planner entry filename, e.g. my_planner.py")
    parser.add_argument(
        "--root-subdir",
        default=None,
        help="subdir under policy/ (default: same as --method)",
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    launch_path = repo_root / "Controller" / "src" / "utils" / "yopo_bringup" / "launch" / "system.launch.py"
    root_subdir = args.root_subdir or args.method
    register_method(launch_path, args.method, args.entry, root_subdir)
    print(f"Registered planner method '{args.method}' in {launch_path}")


if __name__ == "__main__":
    main()
