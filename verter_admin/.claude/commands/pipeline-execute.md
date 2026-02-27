Implement the selected phase by applying code changes to the repository.

**Arguments:** `$ARGUMENTS` is the phase number (e.g. `1`, `2`, `3`, `4`).

---

## Step 1 — Load context

If `$ARGUMENTS` is empty, ask the user for the phase number before proceeding.

Find the phase brief:
```bash
ls -1dt docs/robot-pipeline/runs/*/implement/phase-$ARGUMENTS-brief.md 2>/dev/null | head -1
```

Read the brief file. If it does not exist, stop and tell the user to run `/pipeline-implement $ARGUMENTS` first.

Read the SDHR rulebook: `.claude/rules/robotics_architecture_rulebook.md`

## Step 2 — Confirm scope with the user

Before touching any file, show the user the `## Scope Boundaries` and `## File-Level Change Plan` sections from the brief and ask for explicit confirmation to proceed.

## Step 3 — Execute

**Hard constraints (non-negotiable):**
- Modify ONLY files listed in the brief's `## File-Level Change Plan`. Any other file is FORBIDDEN.
- Do NOT bypass or weaken the safety-gated command path.
- Preserve dependency direction: `domain → application → adapters → infrastructure`. FORBIDDEN: domain importing adapters or infrastructure.
- Do NOT commit.
- Apply the smallest safe change set — no speculative improvements outside the brief scope.
- Keep all configuration and runtime behaviour fail-safe by default.

If any requested change conflicts with a mandatory SDHR constraint, STOP immediately and report the conflict to the user instead of proceeding.

Work through the `## File-Level Change Plan` items in order. For each file: read the current state, apply the change, verify the change matches the brief intent.

## Step 4 — Verify

Run both checks:
```bash
python3 -m compileall -q src/verter_admin
./services/verify_mvp.sh
```

If either check fails, fix the issue before reporting. Do not leave the codebase in a broken state.

## Step 5 — Write result file and report

Write `{run_dir}/implement/phase-$ARGUMENTS-execute-result.md` with:

```
# Execute Result — Phase $ARGUMENTS

- executed_at_utc: <current UTC time>
- verification: <pass/fail>

## Changed Files

<list every modified file with one-line reason>

## Planned vs Actual

<table: File | Planned Change (from brief) | Actual Change | Match (yes/diff)>

## Verification Status

<pass/fail for each check>

## Unresolved Risks

<items from brief § Risks and Assumptions still open>

## Forbidden Actions Confirmation

- Files outside brief scope modified: NO
- Safety-gated command path bypassed: NO
- Dependency direction violated: NO
- Commit made: NO
```

Show the user a concise summary of the result file contents.
