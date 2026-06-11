# Safety-Driven Hexagonal Robotics (SDHR) Rulebook

Version: 1.0  
Status: Normative  
Audience: AI agents and engineers producing robotics design/plan/implementation artifacts.

## 1. Normative Language

- `MUST`: mandatory requirement.
- `SHOULD`: strong recommendation; deviations require explicit justification.
- `MAY`: optional.
- `FORBIDDEN`: not allowed.

## 2. Architecture Vision

The target architecture is a hybrid of:
- layered robotics system decomposition (clear module roles and interfaces),
- hexagonal boundaries inside each module/package,
- safety-first command arbitration as a non-optional envelope.

### 2.1 Layered Decomposition by Criticality

- `L0 Safety/Actuation` (MCU and immediate hardware control path)
  - MUST own emergency stopping and command timeout behavior.
  - MUST remain functional without mission/planning stack.
- `L1 Control/Localization`
  - MUST provide deterministic control and state estimation services.
- `L2 Planning/Behavior`
  - SHOULD be replaceable without changing L0/L1 contracts.
- `L3 Mission/HMI/Cloud`
  - MUST NOT be in direct actuator control path.

Inter-layer dependency rules:
- L2 and L3 MUST communicate with L0/L1 exclusively through defined L1 contracts (ROS topics/services/actions).
- `FORBIDDEN`: L2 or L3 issuing commands directly to L0 hardware, bypassing L1 safety contracts.
- `FORBIDDEN`: L0 depending on L2/L3 for normal operation or fault recovery.
- L1 MUST NOT import or call L2/L3 code at runtime.

### 2.2 Hexagonal Boundaries (per package/module)

Each package MUST be split conceptually as:
- `domain`: invariants, state machine, policies.
- `application`: use-cases orchestrating domain.
- `adapters`: ROS topics/services/actions, drivers, protocol glue.
- `infrastructure`: launch, runtime wiring, config.

Dependency rule:
- domain -> (none)
- application -> domain
- adapters -> application/domain interfaces
- infrastructure -> adapters/application
- `FORBIDDEN`: domain importing adapters/infrastructure.

## 3. Command & Safety Envelope

### 3.1 Single Safety-Gated Command Chain

- All actuator commands MUST pass through a safety-gated arbitration chain.
- There MUST be exactly one final actuator ingress interface (single source of truth).
- `FORBIDDEN`: direct bypass command path around safety gate.

### 3.2 Priority & Timeout

- Emergency/safety command source MUST have higher priority than teleop/autonomy.
- Command timeout MUST trigger safe stop.
- Recovery from stop MUST be explicit and observable.

### 3.3 Invariants

At minimum, design MUST include these invariants:
- E-stop precedence.
- Watchdog timeout to safe stop.
- Bounded velocity/acceleration/jerk limits. Enforcement MUST be explicit at L0 (MCU) as the last line of defense; L1 MAY add a redundant software layer but MUST NOT rely on L1 alone.
- Sensor staleness handling.
- Fail-safe default on startup and unknown states.
- Deterministic degraded mode behavior.

## 4. Lifecycle, Bringup, and Fault Recovery

- Managed lifecycle MUST be used for any node that owns actuator commands, safety state, or sensor fusion. Nodes excluded from managed lifecycle MUST carry an explicit documented justification.
- Bringup order MUST ensure sensor/data providers are available before dependent consumers.
- Shutdown order SHOULD be reverse dependency order.
- Fault transitions MUST be explicit (`ACTIVE -> DEGRADED/FAULT`).
- Recovery policy MUST define who can reset and under which checks.

## 5. Interface & Data Contracts

For every interface in design artifacts, include:
- owner, producers, consumers,
- message schema and units (SI where applicable),
- coordinate frame semantics (`frame_id` rules),
- QoS profile and rationale,
- expected rate/latency/timeout,
- failure behavior on stale/missing data.

`FORBIDDEN`:
- hidden implicit contracts,
- unbounded topic rename/refactor without migration plan,
- mixed units in same contract.

## 6. QoS Policy Baseline

- Sensor streams SHOULD favor freshness (best effort profile where acceptable).
- Commands/control-critical paths MUST use reliable delivery with strict depth/timeouts. Loss of a command on the actuator path is a safety failure.
- Services MUST avoid stale replay behavior (volatile semantics expected for request/response flows).
- QoS compatibility MUST be checked for pub/sub pairs.

## 7. Real-Time & Determinism Baseline

- Critical path MUST avoid unbounded dynamic allocation in runtime loops.
- Memory and scheduling policy SHOULD be defined for low-jitter control loops.
- Execution ordering for safety-critical callbacks MUST be deterministic by design.
- On MCU/micro-ROS side, post-init runtime SHOULD avoid dynamic allocation and use deterministic executor semantics.

## 8. TF / Frames / Semantics

- Coordinate frames MUST follow REP conventions for mobile robots.
- Every spatial message MUST carry correct and documented frame semantics.
- Transform tree ownership MUST be explicit.

## 9. Security & Operational Hardening

- Secrets and credentials MUST NOT be hardcoded.
- Security boundaries/enclaves SHOULD be explicit for production deployments.
- Launch/runtime security configuration MUST be reviewable and reproducible.

## 10. Quality Process Baseline

Target minimum: REP-2004 Quality Level 2 style process.

Minimum expectations:
- change control via PR/MR,
- CI gates for each change,
- static analysis and style enforcement,
- unit tests for domain layer logic (domain MUST be testable without ROS or hardware dependencies),
- integration tests for node behaviour (with mock interfaces),
- system/integration tests for documented features,
- documented dependency rationale,
- vulnerability disclosure path.

## 11. Output Contracts for AI Artifacts

### 11.1 Design Artifact MUST include

1. C4 system context
2. Container/component architecture
3. Data flow (command + feedback)
4. State machine
5. Safety invariants table
6. QoS & interface contract matrix
7. Failure modes and recovery table
8. Open decisions and assumptions
9. Constraint compliance matrix

### 11.2 Plan Artifact MUST include

1. Quality gates
2. Phase-by-phase plan with measurable acceptance criteria
3. Traceability matrix (`constraint -> phase -> test`)
4. Explicit artifacts per phase
5. Risk register and rollback strategy
6. Constraint compliance matrix

### 11.3 Implement Brief MUST include

1. Phase scope boundaries
2. File-level change plan
3. Constraint checklist
4. Verification steps
5. Risks/assumptions
6. Out-of-scope items

## 12. Rejection Criteria (hard fail)

Artifact MUST be rejected if any condition is true:
- safety-gated command chain is bypassed or undefined,
- lifecycle/fault behavior is missing for critical modules,
- interface contracts lack QoS/timeout/frame semantics,
- no traceability from constraints to tests,
- architecture violates dependency direction,
- constraints are not acknowledged in compliance matrix.

## 13. Primary Source References

- ROS 2 Managed Lifecycle: https://design.ros2.org/articles/node_lifecycle.html
- ROS 2 QoS design: https://design.ros2.org/articles/qos
- ROS 2 Real-time background: https://design.ros2.org/articles/realtime_background.html
- ROS 2 Composition concepts: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Composition.html
- micro-ROS features: https://micro.ros.org/docs/overview/features/
- micro-ROS execution management: https://micro.ros.org/docs/concepts/client_library/execution_management/
- micro-ROS executor tutorial: https://micro.ros.org/docs/tutorials/programming_rcl_rclc/executor/
- Nav2 lifecycle manager behavior/order: https://docs.nav2.org/tutorials/docs/adding_a_nav2_task_server.html
- ros2_control controller manager determinism: https://docs.ros.org/en/rolling/p/controller_manager/doc/userdoc.html
- ros2_control framework docs: https://control.ros.org/master/doc/ros2_control/doc/index.html
- tf2 and frame convention reference to REP-105: https://docs.ros.org/en/rolling/p/tf2/__README.html
- REP-105 coordinate frames: https://www.ros.org/reps/rep-0105.html
- REP-2004 quality levels: https://www.ros.org/reps/rep-2004.html
- Autoware layered architecture rationale: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/
