You are a senior robotics software engineer. Produce a strict IMPLEMENTATION BRIEF for one selected phase.

**Arguments:** `$ARGUMENTS` is the phase number (e.g. `1`, `2`, `3`, `4`).

---

## Step 1 — Load context

If `$ARGUMENTS` is empty, ask the user for the phase number before proceeding.

Find the latest run directory:
```bash
ls -1dt docs/robot-pipeline/runs/*/ 2>/dev/null | head -1
```

Read `{run_dir}/03_plan.md`. If it does not exist, stop and tell the user to run `/pipeline-plan` first.

Extract the `## Phase $ARGUMENTS` section from the plan (from that heading until the next `## Phase` heading or end of file).
If the phase section is not found, stop and report the available phases.

Read the SDHR rulebook: `.claude/rules/robotics_architecture_rulebook.md`

## Step 2 — Generate the brief

Using the selected phase section, the full plan, and the rulebook as mandatory context, produce a complete IMPLEMENT BRIEF.

**Hard constraints:**
- Output Markdown only.
- Keep changes strictly within the selected phase boundaries.
- Maintain SDHR architecture and all safety invariants.
- Explicitly list what is FORBIDDEN for this phase.
- Use `MUST`, `SHOULD`, `FORBIDDEN`.

**Required headings (use exactly these, in this order):**

```
# Implement Phase $ARGUMENTS
## Scope Boundaries
## File-Level Change Plan
## Constraint Checklist
## Verification Steps
## Risks and Assumptions
## Out of Scope
```

**Content requirements:**

- `## Scope Boundaries`: restate the phase goal and exact boundaries — what systems, packages, and files are in scope. One sentence per boundary. Reference the relevant sections of `02_design.md` (e.g. "implements safety invariants S3, S4 from Design §Safety Invariants").

- `## File-Level Change Plan`: table or itemised list with columns `File | Change Type (add/modify/delete) | Reason in scope`. Every file that will be touched MUST be listed. Files not listed are FORBIDDEN to touch.

- `## Constraint Checklist`: itemised checklist of SDHR constraints that apply to this phase. Each item MUST be checkable (pass/fail). Include at minimum:
  - MUST keep command chain safety-first and unbroken
  - FORBIDDEN bypass around safety envelope
  - MUST keep dependency direction: domain → application → adapters → infrastructure
  - FORBIDDEN: domain importing adapters or infrastructure
  - `python3 -m compileall -q src/verter_admin` must pass
  - Include a mini traceability table: `Constraint | File Change | Test`

- `## Verification Steps`: ordered list of steps to verify the phase is done. Each step must be concrete (a command to run, a file to inspect, a behaviour to observe).

- `## Risks and Assumptions`: itemised list of risks with likelihood/impact, and explicit assumptions made.

- `## Out of Scope`: explicit list of items that are NOT part of this phase, including items that may seem related but are deferred.

## Step 3 — Write and validate

Write the generated brief to `{run_dir}/implement/phase-$ARGUMENTS-brief.md`.

Verify the file contains all required headings: `Scope Boundaries`, `File-Level Change Plan`, `Constraint Checklist`, `Verification Steps`, `Risks and Assumptions`, `Out of Scope`. If any are missing, fix and rewrite.

## Step 4 — Report

Show the user:
- Path to the brief file
- List of files in scope for this phase
- Validation result
- Next step: `/pipeline-execute $ARGUMENTS`
