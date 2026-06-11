You are a robotics systems architect. Produce a normative DESIGN artifact for a ROS2 + micro-ROS system.

---

## Step 1 — Load context

Find the latest run directory:
```bash
ls -1dt docs/robot-pipeline/runs/*/ 2>/dev/null | head -1
```

Read `{run_dir}/01_research.md`. If it does not exist, stop and tell the user to run `/pipeline-research` first.

Read the SDHR rulebook: `.claude/rules/robotics_architecture_rulebook.md`

## Step 2 — Generate the design

Using the research and rulebook as mandatory context, produce a complete DESIGN artifact.

**Hard constraints:**
- Output Markdown only.
- Use `MUST`, `SHOULD`, `FORBIDDEN` (RFC-2119) throughout.
- Do not invent facts not present in the research. Use explicit assumptions instead of guessing.
- The SDHR rulebook is mandatory — every requirement in it applies.

**Required headings (use exactly these, in this order):**

```
# Design
## C4 System Context
## Container/Component Architecture
## Data Flow
## State Machine
## Safety Invariants
## QoS and Interface Contract Matrix
## Failure Modes and Recovery
## Open Decisions and Assumptions
## Constraint Compliance Matrix
```

**Content requirements:**
- `## C4 System Context`: actors, external systems, context diagram (Mermaid flowchart).
- `## Container/Component Architecture`: L0/L1/L2/L3 layer breakdown; hexagonal split (domain / application / adapters / infrastructure) per package; explicitly mark what is FORBIDDEN in each.
- `## Data Flow`: Mermaid diagram of command path from operator/nav2 → safety gate → actuator, and sensor feedback path.
- `## State Machine`: Mermaid stateDiagram covering at minimum: INIT → READY → ACTIVE → DEGRADED → FAULT, with all transitions and triggers labelled.
- `## Safety Invariants`: table with columns `ID | Invariant | Enforcement Layer | Verification Strategy`. Minimum 8 invariants (E-stop, watchdog, bounded velocity at L0, sensor staleness, fail-safe startup, deterministic degraded, manual override auditability, single command ingress).
- `## QoS and Interface Contract Matrix`: table with columns `Interface | Owner | Producers | Consumers | QoS Profile | Rate | Timeout | MUST/FORBIDDEN`. No TBD values — use explicit assumptions if data is missing.
- `## Failure Modes and Recovery`: table with columns `Failure | Detection | State Transition | Recovery Path`.
- `## Open Decisions and Assumptions`: itemised list of unresolved decisions and explicit assumptions made.
- `## Constraint Compliance Matrix`: table mapping every SDHR constraint from the rulebook to its status (COMPLIANT / PARTIAL / N/A) and evidence section.
- Include at least 5 explicit FORBIDDEN moves across relevant sections.

## Step 3 — Write and validate

Write the generated design to `{run_dir}/02_design.md`.

Verify the file contains all required headings and the tokens `MUST` and `FORBIDDEN`. If any are missing, fix and rewrite before proceeding.

## Step 4 — Report

Show the user:
- Path to `02_design.md`
- Validation result (headings present, MUST/FORBIDDEN used)
- Next step: `/pipeline-plan`
