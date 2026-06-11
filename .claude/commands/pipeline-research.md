Start a new robot pipeline run and create the research artifact.

**Arguments:** `$ARGUMENTS` is the task description.

---

## Step 1 — Prepare

If `$ARGUMENTS` is empty, ask the user for the task description before proceeding.

Create the run directory and write the task file:

```bash
SLUG=$(printf "%s" "$ARGUMENTS" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9]+/-/g; s/^-+//; s/-+$//' | cut -c1-50)
RUN_DIR="docs/robot-pipeline/runs/$(date -u +%Y%m%d-%H%M%S)-${SLUG:-task}"
mkdir -p "${RUN_DIR}/artifacts" "${RUN_DIR}/implement"
echo "${RUN_DIR}"
```

Capture the printed path as `RUN_DIR` for all subsequent steps.

Write `${RUN_DIR}/00_task.md`:

```
# Task

- created_at_utc: <current UTC time>
- source: inline

## Goal

<$ARGUMENTS>
```

## Step 2 — Scan mode

Default mode is **project-facts** (scans the live codebase). Use **best-practice-only** only if the user explicitly requests it (greenfield task, no relevant code yet). Do not ask — proceed with project-facts unless told otherwise.

## Step 3 — Project-facts scan

Run each query and capture all output. If a directory does not exist, the command produces no output — note "(none found)" in that case.

```bash
# Node inventory (ROS2 node constructors)
rg -n "super\(\)\.__init__\('([^']+)'" src/verter_admin \
  | sed -E "s#^([^:]+):([0-9]+):.*'([^']+)'.*#- \3 (\1:\2)#" 2>/dev/null || true

# Topic map (publishers and subscriptions)
rg -n "create_(publisher|subscription)\([^,]+,\s*'[^']+'" src/verter_admin \
  | sed -E "s#^([^:]+):([0-9]+):(.*)#- \1:\2 ::\3#" 2>/dev/null || true

# Command flow references
rg -n "/teleop_keyboard/cmd_vel|/nav2/cmd_vel|/safety/cmd_vel|/cmd_vel|twist_mux|micro_ros_agent" \
  src/verter_admin/launch src/verter_admin/control 2>/dev/null || true

# Driver and hardware references
rg -n "micro_ros_agent|rplidar|/dev/esp32|/dev/rplidar|wheel_encoders|imu/data|ultrasonic" \
  src/verter_admin/launch firmware src/verter_admin/config 2>/dev/null || true

# Safety parameters
rg -n "stop_distance|resume_distance|sensor_timeout_sec|control_rate_hz|watchdog|failsafe|safety" \
  src/verter_admin/control src/verter_admin/launch src/verter_admin/config 2>/dev/null || true

# Launch files
ls src/verter_admin/launch/*.launch.py 2>/dev/null || true

# Key YAML config files and notable parameter values
ls src/verter_admin/config/**/*.yaml 2>/dev/null || true
rg -n "robot_radius|inflation_radius|max_vel_x|max_vel_theta|update_frequency|frequency|timeout" \
  src/verter_admin/config 2>/dev/null || true

# Python package entry points
rg -n "entry_points|console_scripts" setup.py 2>/dev/null || true
```

## Step 4 — Write 01_research.md

Write `${RUN_DIR}/01_research.md` with these sections:

- Header metadata: `created_at_utc`, `run_dir`, `mode`
- `## Task` — copy of goal from `00_task.md`
- `## Baseline Architecture Assumptions` — SDHR style, L0/L1/L2/L3 layering, single safety-gated command chain is mandatory, contracts and verification traceability are first-class artifacts
- `## Mandatory Constraint Set`:
  - MUST define lifecycle and fault transitions for critical nodes
  - MUST define QoS, timeout, ownership, units, and frame semantics for every interface
  - MUST trace every safety invariant to a concrete test
  - FORBIDDEN: bypass around safety envelope command chain
  - FORBIDDEN: dependency direction inversion violating SDHR (including inter-layer: L2/L3 bypassing L1)
- `## Node Inventory`, `## Topic Map`, `## Command Flow References`, `## Driver and Hardware References`, `## Safety-Related Parameters`, `## Launch Files`, `## Key Config Parameters`, `## Entry Points` — populated with scan results; write "(none found)" explicitly if a query returned nothing
- `## Known Unknowns` — items to clarify in Design (command contracts, recovery policy, thresholds, security boundaries)

## Step 5 — Report

Show the user:
- Run directory path
- Brief summary of key facts captured (nodes found, topics, safety refs, config files)
- Next step: `/pipeline-design`
