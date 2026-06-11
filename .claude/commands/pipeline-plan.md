You are a robotics technical lead. Produce a phase-by-phase IMPLEMENTATION PLAN aligned with the SDHR rulebook.

---

## Step 1 — Load context

Find the latest run directory:
```bash
ls -1dt docs/robot-pipeline/runs/*/ 2>/dev/null | head -1
```

Read `{run_dir}/02_design.md`. If it does not exist, stop and tell the user to run `/pipeline-design` first.

Read the SDHR rulebook: `.claude/rules/robotics_architecture_rulebook.md`

## Step 2 — Generate the plan

Using the design and rulebook as mandatory context, produce a complete IMPLEMENTATION PLAN.

**Hard constraints:**
- Output Markdown only.
- Phases MUST be independently reviewable and testable.
- Every phase MUST include objective acceptance criteria.
- Every constraint MUST be traceable to a concrete test.
- Include rollback strategy and explicit risk ownership.
- Use `MUST`, `SHOULD`, `FORBIDDEN`.

**Required headings (use exactly these, in this order):**

```
# Implementation Plan
## Quality Gates
## Phase 1
## Phase 2
[## Phase 3]
[## Phase 4]
## Traceability Matrix
## Risk Register and Rollback
## Constraint Compliance Matrix
```

Use as many phases as the design requires. Minimum 2, maximum 4. Do not create empty phases to fill a quota.

**Content requirements:**

- `## Quality Gates`: list of pass/fail checks that apply before any phase is considered done. At minimum: Python syntax check (`python3 -m compileall -q src/verter_admin`), static/safety conformance check, architecture dependency direction check, unit tests for domain layer pass.

- `## Phase N` (each phase): use this sub-structure for every phase:
  - **Goal**: one sentence.
  - **Scope boundaries**: what is in scope and what is explicitly out of scope.
  - **Deliverables**: concrete files or artifacts produced.
  - **Tests**: how the deliverables are verified (include unit tests for domain changes).
  - **Exit criteria**: MUST-level conditions that gate moving to the next phase.

  Suggested phase breakdown (adapt to the design — use fewer phases if the task is smaller):
  - Phase 1: lock contracts and interface boundaries (no behavioral changes).
  - Phase 2: safety envelope invariants and fault behavior.
  - Phase 3: enforce hexagonal dependency direction.
  - Phase 4: integration wiring, launch/config hardening.

- `## Traceability Matrix`: table with columns `Constraint | Phase(s) | Test/Artifact`. Every SDHR constraint from the rulebook MUST appear.

- `## Risk Register and Rollback`: table with columns `Risk | Likelihood | Impact | Owner | Rollback Trigger | Rollback Path`.

- `## Constraint Compliance Matrix`: table with columns `Constraint | Status | Evidence`. Map every SDHR constraint to a phase and artifact.

## Step 3 — Write and validate

Write the generated plan to `{run_dir}/03_plan.md`.

Ensure `{run_dir}/artifacts/` directory exists:
```bash
mkdir -p "${run_dir}/artifacts"
```

Verify the file contains headings: `Quality Gates`, `Traceability Matrix`, `Constraint Compliance Matrix`, `Risk Register and Rollback`, and at least `## Phase 1` and `## Phase 2`. If any are missing, fix and rewrite.

## Step 4 — Report

Show the user:
- Path to `03_plan.md`
- Number of phases generated
- Validation result
- Next step: `/pipeline-implement 1`
