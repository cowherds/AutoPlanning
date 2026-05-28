# New Algorithm Development Workflow

This document defines the end-to-end workflow for adding a new planner algorithm under `policy/`.

It is designed for the current project structure:

- algorithm code lives in `policy/<algo_name>/`
- startup and test scripts live in `policy/<algo_name>/script/`
- visualization assets are centralized in `visualization/`

---

## 1. Create the new algorithm folder

From repository root:

```bash
cd /home/duckcity/YOPO_lidar_V3
mkdir -p policy/<algo_name>/script
```

Recommended layout:

```text
policy/<algo_name>/
  script/
    test_<algo_name>.py
    run_<algo_name>_ros2.sh
  config/
    config.py
    *.yaml
  <algo_core_files>.py
```

Rules:

- Put algorithm core files directly under `policy/<algo_name>/`.
- Keep `policy/<algo_name>/script/` only for launcher or test entry scripts.

---

## 2. Implement algorithm runtime node

Create your main runtime entry, for example:

- `policy/<algo_name>/<algo_name>_planner.py`

Your node should:

- subscribe odometry topic (default `/sim/odom`)
- subscribe lidar topic (default `/lidar_points`)
- publish control topic (default `/so3_control/pos_cmd`)
- expose parameters by ROS2 args if possible

Use the same topic contract as existing planners to simplify integration.

---

## 3. Add local startup scripts

Create:

- `policy/<algo_name>/script/test_<algo_name>.py`
- `policy/<algo_name>/script/run_<algo_name>_ros2.sh`

`run_<algo_name>_ros2.sh` should:

1. source ROS2 (`/opt/ros/humble/setup.bash`)
2. source controller workspace setup
3. execute your planner entry script

Keep script behavior similar to existing `policy/YOPO/script/run_yopo_ros2.sh`.

---

## 4. Integrate into system launch

Use the auto-registration script (no manual launch editing):

```bash
cd /home/duckcity/YOPO_lidar_V3
python scripts/register_planner_method.py \
  --method <algo_name> \
  --entry <algo_name>_planner.py
```

Optional if your folder name differs from method name:

```bash
python scripts/register_planner_method.py \
  --method <planner_method_value> \
  --root-subdir <policy_subdir_name> \
  --entry <planner_entry_file.py>
```

What the script automatically updates in `system.launch.py`:

1. add a new `planner_method` branch value
2. add `<algo_name>_root` launch argument with default `policy/<algo_name>`
3. add an `ExecuteProcess` branch for your planner entry
4. include your new argument and node in `LaunchDescription([...])`

---

## 5. Keep visualization centralized

Use centralized RViz configs in:

- `visualization/yopo_ros2.rviz`
- `visualization/yopo.rviz`

Run:

```bash
cd /home/duckcity/YOPO_lidar_V3
rviz2 -d visualization/yopo_ros2.rviz
```

If your algorithm needs extra topics, update or add a profile under `visualization/`.

---

## 6. Validation workflow

### Step A: Unit-level local test

Run only your planner script and verify:

- node starts
- parameters load
- topics are created

### Step B: Planner-only ROS test

Launch planner-only mode (or your test script), check:

```bash
ros2 topic info /sim/odom -v
ros2 topic info /lidar_points -v
ros2 topic info /so3_control/pos_cmd -v
```

### Step C: Full stack test

Use:

```bash
ros2 launch yopo_bringup system.launch.py planner_method:=<algo_name>
```

Then publish one goal and verify control outputs.

---

## 7. Common pitfalls

1. **Working directory dependent paths**
   - Always resolve model/config paths robustly (`Path(__file__)` based).
2. **Weight incompatibility**
   - Check checkpoint keys match model structure.
3. **Multiple publishers conflict**
   - Ensure only one planner publishes `/so3_control/pos_cmd`.
4. **Un-sourced ROS workspaces**
   - Source ROS2 and controller setup before launching.
5. **Hardcoded old directory names**
   - Use `policy/<algo_name>` paths consistently in launch and docs.

---

## 8. Minimal checklist before merge

- [ ] `policy/<algo_name>/` exists with clear structure
- [ ] `script/` contains only startup/test scripts
- [ ] planner integrated in `system.launch.py`
- [ ] topics follow project contract
- [ ] visualization config validated
- [ ] docs updated with run command and expected inputs

