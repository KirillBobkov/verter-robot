Show the robot pipeline usage guide.

Print the following instructions to the user verbatim:

---

## Robot Pipeline — Usage Guide

The pipeline turns a task description into reviewed, validated code changes through 5 sequential steps. All logic is self-contained in the slash commands — no external scripts required. The only external file is the SDHR rulebook at `.claude/rules/robotics_architecture_rulebook.md`.

### Full sequence

```
/pipeline-research <task description>
/pipeline-design
/pipeline-plan
/pipeline-implement 1
/pipeline-execute 1
# repeat implement + execute for phases 2, 3, 4 ...

/pipeline-status   ← check progress at any point
```

---

### Commands

#### `/pipeline-research <task>`

Starts a new pipeline run. Optionally scans the codebase and produces a research snapshot.

```
/pipeline-research add proximity-based emergency stop to the control layer
```

- Creates a timestamped run directory under `docs/robot-pipeline/runs/`.
- Produces `01_research.md`.
- Asks whether to scan the live codebase (**project-facts**) or use SDHR baseline only (**best-practice-only**).
- **Run this first** before any other step.

---

#### `/pipeline-design`

Acts as a robotics systems architect. Reads the research and produces a full system design following the SDHR rulebook.

```
/pipeline-design
```

- Reads `01_research.md` and the SDHR rulebook.
- Produces `02_design.md` with: C4 context, component architecture, data flow, state machine, safety invariants, QoS contract matrix, failure modes, compliance matrix.
- Validates all required headings before finishing.

---

#### `/pipeline-plan`

Acts as a robotics technical lead. Reads the design and produces a phase-by-phase implementation plan.

```
/pipeline-plan
```

- Reads `02_design.md` and the SDHR rulebook.
- Produces `03_plan.md` with up to 4 phases, each with: goal, scope, deliverables, tests, exit criteria.
- Includes traceability matrix and risk register.
- Validates all required headings before finishing.

---

#### `/pipeline-implement <phase>`

Acts as a senior robotics software engineer. Produces a strict implementation brief for one phase.

```
/pipeline-implement 1
/pipeline-implement 2
```

- Reads `03_plan.md`, extracts the selected phase, reads the SDHR rulebook.
- Produces `implement/phase-N-brief.md` with: scope boundaries, file-level change plan, constraint checklist, verification steps, risks, out-of-scope items.
- Validates all required headings before finishing.
- **Run this before `/pipeline-execute` for each phase.**

---

#### `/pipeline-execute <phase>`

Applies the code changes described in the brief.

```
/pipeline-execute 1
```

- Reads the brief from `/pipeline-implement`.
- **Asks for explicit confirmation** before touching any file.
- Modifies only files listed in the brief.
- Runs `python3 -m compileall` and `./services/verify_mvp.sh` after changes.
- Stops and reports if any change conflicts with an SDHR constraint.
- Does **not** commit — review and commit manually.

---

#### `/pipeline-status`

Shows the current state of the latest pipeline run.

```
/pipeline-status
```

- Displays which artifacts exist and which are missing.
- Recommends the next command to run.
- Lists any older runs.

---

### Artifacts per run

```
docs/robot-pipeline/runs/<timestamp>-<slug>/
  00_task.md                        ← task description
  01_research.md                    ← codebase facts or SDHR baseline
  02_design.md                      ← system design
  03_plan.md                        ← implementation plan
  artifacts/                        ← phase-level artifact files
  implement/
    phase-1-brief.md                ← implement brief
    phase-1-execute-result.md       ← execution result
    phase-2-brief.md                ← ...
    phase-2-execute-result.md
```

### Working with multiple runs

All commands use the **latest** run directory (by modification time under `docs/robot-pipeline/runs/`).
To start a fresh run on a new task, run `/pipeline-research <new task>` — this creates a new timestamped directory and becomes the new "latest".

### Tips

- Always run steps in order — each step depends on the previous artifact.
- Review each artifact before proceeding to the next step.
- `/pipeline-execute` is the only command that modifies source files — all others are read-only.
- The SDHR rulebook is loaded by every command automatically from `.claude/rules/robotics_architecture_rulebook.md`.
