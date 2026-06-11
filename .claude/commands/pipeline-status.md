Show the current state of the robot pipeline.

---

## Step 1 — Find the latest run

```bash
ls -1dt docs/robot-pipeline/runs/*/ 2>/dev/null | head -1
```

If no run directories exist, tell the user there are no runs yet and suggest `/pipeline-research <task>`.

## Step 2 — Check artifact state

For the latest run directory, check which files exist:

```bash
# Core artifacts
ls -1 {run_dir}/00_task.md \
       {run_dir}/01_research.md \
       {run_dir}/02_design.md \
       {run_dir}/03_plan.md 2>/dev/null

# Phase briefs and results
ls -1 {run_dir}/implement/phase-*-brief.md \
       {run_dir}/implement/phase-*-execute-result.md 2>/dev/null || true
```

Also read the task goal from `{run_dir}/00_task.md` if it exists.

## Step 3 — Report

Print a status table like this:

```
Run: docs/robot-pipeline/runs/<timestamp>-<slug>
Task: <goal from 00_task.md>

Step                  File                     Status
----                  ----                     ------
1. Research           01_research.md           ✓ done  / ✗ missing
2. Design             02_design.md             ✓ done  / ✗ missing
3. Plan               03_plan.md               ✓ done  / ✗ missing
4. Implement Phase 1  implement/phase-1-brief  ✓ done  / ✗ missing
   Execute Phase 1    implement/phase-1-result ✓ done  / ✗ missing
5. Implement Phase 2  implement/phase-2-brief  ✓ done  / ✗ missing
   Execute Phase 2    implement/phase-2-result ✓ done  / ✗ missing
...
```

Then show the next recommended action based on the first missing step:
- If `01_research.md` missing → `/pipeline-research <task>`
- If `02_design.md` missing → `/pipeline-design`
- If `03_plan.md` missing → `/pipeline-plan`
- If a brief is missing → `/pipeline-implement <N>`
- If a brief exists but result is missing → `/pipeline-execute <N>`
- If everything is done → "All phases complete. Review artifacts and commit."

Also list any other run directories (not the latest) so the user can see if there are older runs:
```bash
ls -1dt docs/robot-pipeline/runs/*/ 2>/dev/null | tail -n +2
```
