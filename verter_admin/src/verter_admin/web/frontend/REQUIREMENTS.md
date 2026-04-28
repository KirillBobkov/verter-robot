# Functional Requirements: Verter Robot Web Interface

**Version:** 1.0
**Status:** Draft
**Date:** 2025-04-24
**Author:** Generated from existing codebase analysis

---

## 1. Introduction

### 1.1 Purpose

The Verter Robot Web Interface is a touchscreen-based web application designed for service robot operation in medical/hospital environments. The interface serves two primary user groups:

1. **Setup Mode** - For engineers and technicians to configure the robot (create maps, set waypoints, calibrate)
2. **Active Mode** - For patients and visitors to interact with the robot (navigation, information, voice assistance)

### 1.2 System Overview

The web interface is a React-based Single Page Application (SPA) that communicates with the ROS 2 robot backend via rosbridge (WebSocket). It runs on a tablet mounted on the robot and provides:

- Real-time robot control and monitoring
- Navigation waypoint management
- Patient-facing guidance interface
- Multilingual support (Russian/English)
- Voice interaction capabilities

### 1.3 Target Users

| User Role | Mode | Description | Technical Level |
|-----------|------|-------------|-----------------|
| Setup Engineer | Setup | Configures robot, creates maps, manages waypoints | High |
| Patient/Visitor | Active | Uses robot for guidance, asks questions | Low |
| Medical Staff | Active | Quick information access, navigation | Medium |

### 1.4 Goals

1. Provide intuitive robot configuration for engineers
2. Enable accessible navigation guidance for patients/visitors
3. Ensure safe robot operation with emergency stop functionality
4. Support bilingual operation (Russian/English)
5. Integrate with ROS 2 Nav2 and SLAM systems

---

## 2. Actors and Their Roles

### 2.1 Setup Engineer (Setup Mode)

**Description:** Technical personnel responsible for robot deployment and maintenance.

**Capabilities:**
- Access Setup Mode interface
- Create and save maps using SLAM
- Set home/charging position
- Add, edit, delete navigation waypoints
- Manually control robot (WASD keyboard)
- Export/import configuration

**Authentication:** None required (local interface)

### 2.2 Patient/Visitor (Active Mode)

**Description:** End users seeking navigation assistance or information.

**Capabilities:**
- View idle screen with greeting
- Select destination from list
- Request guided navigation
- Access information screens
- Ask questions via voice
- Change language preference

**Authentication:** None required (kiosk mode)

### 2.3 System (Automated)

**Description:** Background processes and ROS 2 integration.

**Capabilities:**
- Monitor activity and return to idle
- Emergency stop on safety events
- Voice activation detection
- Navigation status tracking
- Connection health monitoring

---

## 3. Functional Requirements by Mode

### 3.1 Setup Mode

#### 3.1.1 Mapping (Map Creation)

**Requirement:** The system shall provide a map creation interface for SLAM-based mapping.

| ID | Description | Priority |
|----|-------------|----------|
| SM-001 | Start SLAM mapping session via button | P0 |
| SM-002 | Stop SLAM mapping session via button | P0 |
| SM-003 | Save created map to persistent storage | P0 |
| SM-004 | Reset/clear current map with confirmation | P1 |
| SM-005 | Display mapping status (active/inactive) | P1 |
| SM-006 | Show live map preview with robot position | P1 |
| SM-007 | Log mapping operations with timestamps | P2 |

**Acceptance Criteria:**
- Engineer can start/stop mapping without errors
- Map is saved to ROS 2 map server location
- Status reflects current mapping state accurately

#### 3.1.2 Home Point Setup

**Requirement:** The system shall allow setting the robot's home/charging position.

| ID | Description | Priority |
|----|-------------|----------|
| SH-001 | Display current robot pose (x, y, theta) from `/amcl_pose` | P0 |
| SH-002 | Save current position as "home" waypoint | P0 |
| SH-003 | Rotate robot left/right via on-screen buttons | P0 |
| SH-004 | Test navigation back to home point | P1 |
|SH-005 | Show robot position on map placeholder | P1 |
| SH-006 | Refresh pose data on demand | P2 |

**Acceptance Criteria:**
- Home waypoint is saved with name "home"
- Position updates at least 1 Hz when subscribed
- Rotation buttons allow fine-tuning orientation

#### 3.1.3 Waypoint Management

**Requirement:** The system shall provide full CRUD operations for navigation waypoints.

| ID | Description | Priority |
|----|-------------|----------|
| SP-001 | List all saved waypoints from ROS service | P0 |
| SP-002 | Add new waypoint with custom name | P0 |
| SP-003 | Delete existing waypoint | P0 |
| SP-004 | Edit waypoint (delete + save new) | P1 |
| SP-005 | Use current robot pose for waypoint coordinates | P1 |
| SP-006 | Manually enter coordinates (x, y, theta) | P1 |
| SP-007 | Export waypoints to file | P2 |
| SP-008 | Import waypoints from file | P2 |

**Acceptance Criteria:**
- Waypoints persist across restarts (stored by ROS backend)
- List updates within 2 seconds of changes
- Delete operation requires implicit confirmation

#### 3.1.4 Manual Control (WASD)

**Requirement:** The system shall provide keyboard-based manual robot control.

| ID | Description | Priority |
|----|-------------|----------|
| SC-001 | Enable/disable keyboard control via button | P0 |
| SC-002 | W key: move forward | P0 |
| SC-003 | S key: move backward | P0 |
| SC-004 | A key: rotate left | P0 |
| SC-005 | D key: rotate right | P0 |
| SC-006 | Space: emergency stop | P0 |
| SC-007 | Adjustable linear speed (0.1 - 0.5 m/s) | P1 |
| SC-008 | Adjustable angular speed | P1 |
| SC-009 | Display current speed settings | P1 |

**Technical Details:**
- Publishes to `/cmd_vel` topic at 10 Hz
- Uses `geometry_msgs/Twist` message
- Automatic stop when control disabled

---

### 3.2 Active Mode

#### 3.2.1 Idle Screen

**Requirement:** The system shall display an attractive idle screen when not in use.

| ID | Description | Priority |
|----|-------------|----------|
| AI-001 | Display robot logo and name "Verter" | P0 |
| AI-002 | Show animated eyes | P0 |
| AI-003 | Display greeting message (localized) | P0 |
| AI-004 | "Start" button to begin interaction | P0 |
| AI-005 | Voice activation hint ("Say hello to start") | P1 |
| AI-006 | Language switcher (RU/EN) | P1 |
| AI-007 | Voice activation trigger from ROS topic | P2 |

**Acceptance Criteria:**
- Auto-transitions to menu on any user interaction
- Greeting is localized in selected language
- Eyes animation runs smoothly (60 FPS)

#### 3.2.2 Main Menu

**Requirement:** The system shall provide a main menu with large, accessible buttons.

| ID | Description | Priority |
|----|-------------|----------|
| AM-001 | Display "Guide to Office" button (icon: pedestrian) | P0 |
| AM-002 | Display "Find Office" button (icon: magnifying glass) | P0 |
| AM-003 | Display "Information" button (icon: info) | P0 |
| AM-004 | Display "Ask a Question" button (icon: microphone) | P0 |
| AM-005 | Display "About Robot" button (icon: robot) | P0 |
| AM-006 | Back button to return to idle | P0 |
| AM-007 | Auto-idle after 15 seconds of inactivity | P1 |

**UI Requirements:**
- Buttons minimum 80x80 mm for accessibility
- Icons + text labels
- High contrast colors

#### 3.2.3 Destination Selection

**Requirement:** The system shall allow users to select a destination from available waypoints.

| ID | Description | Priority |
|----|-------------|----------|
| AS-001 | Display all available waypoints as buttons | P0 |
| AS-002 | Assign appropriate icons based on name keywords | P1 |
| AS-003 | Icon mapping: cardiologist→hospital, lab→microscope, etc. | P1 |
| AS-004 | Show "Loading..." state while fetching waypoints | P1 |
| AS-005 | Show "No offices available" if list empty | P1 |
| AS-006 | Back button to main menu | P0 |
| AS-007 | Auto-idle after 15 seconds | P1 |

**Icon Mapping Table:**

| Keyword | Icon |
|---------|------|
| cardiologist/cardio | hospital |
| laboratory/lab | microscope |
| reception/reception | building |
| x-ray/radiology | x-ray |
| surgery/hospital | hospital |
| pharmacy/pharm | pill |
| procedure | syringe |
| doctor | person health |
| (default) | pushpin |

#### 3.2.4 Confirmation Screen

**Requirement:** The system shall confirm the selected destination before navigation.

| ID | Description | Priority |
|----|-------------|----------|
| AC-001 | Display selected destination name | P0 |
| AC-002 | Show confirmation message "I will guide you, follow me" | P0 |
| AC-003 | "Start" button to begin navigation | P0 |
| AC-004 | "Cancel" button to return to selection | P0 |
| AC-005 | Auto-idle timeout enabled | P1 |

#### 3.2.5 Navigation Screen

**Requirement:** The system shall display navigation progress while guiding the user.

| ID | Description | Priority |
|----|-------------|----------|
| AN-001 | Display "Guiding you..." message | P0 |
| AN-002 | Show "Follow me" instruction | P0 |
| AN-003 | Display destination name with arrow | P0 |
| AN-004 | Show mini-map with robot position | P1 |
| AN-005 | Display progress percentage | P1 |
| AN-006 | Progress bar visualization | P1 |
| AN-007 | Emergency stop always available | P0 |
| AN-008 | Auto-idle DISABLED during navigation | P0 |
| AN-009 | Manual "We arrived" button for testing | P2 |

**Navigation States:**
- Planning: "Calculating route..."
- Navigating: Show progress
- Arrived: Transition to ArrivedPage
- Failed: Show error, return to menu

#### 3.2.6 Arrival Screen

**Requirement:** The system shall confirm arrival at destination.

| ID | Description | Priority |
|----|-------------|----------|
| AA-001 | Display checkmark/success icon | P0 |
| AA-002 | Show "We have arrived!" message | P0 |
| AA-003 | Display destination name | P0 |
| AA-004 | Auto-return to idle after 5 seconds | P0 |
| AA-005 | Manual "Return to Start" button | P1 |

#### 3.2.7 Find Office Screen

**Requirement:** The system shall help users locate offices.

| ID | Description | Priority |
|----|-------------|----------|
| AF-001 | Display "Where is the office?" prompt | P0 |
| AF-002 | Office selection interface | P0 |
| AF-003 | "Guide me" button for selected office | P0 |
| AF-004 | Integration with SelectPage for destination | P1 |

#### 3.2.8 Information Screen

**Requirement:** The system shall display general information.

| ID | Description | Priority |
|----|-------------|----------|
| AI-101 | Display working hours | P1 |
| AI-102 | Display visit rules | P1 |
| AI-103 | Floor directory | P2 |
| AI-104 | Contact information | P2 |

#### 3.2.9 Voice Question Screen

**Requirement:** The system shall accept voice questions from users.

| ID | Description | Priority |
|----|-------------|----------|
| AV-001 | Display "Listening..." indicator | P1 |
| AV-002 | Display recognized text | P1 |
| AV-003 | "Repeat question" button | P1 |
| AV-004 | "Finish" button to exit | P1 |
| AV-005 | Integration with ROS speech recognition | P2 |

#### 3.2.10 About Robot Screen

**Requirement:** The system shall display robot information.

| ID | Description | Priority |
|----|-------------|----------|
| AA-101 | Display robot name and description | P1 |
| AA-102 | "Show me how you work" button | P2 |
| AA-103 | "Ask a question" button | P2 |

---

## 4. Non-Functional Requirements

### 4.1 Performance

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-001 | Page load time | < 2 seconds |
| NFR-002 | Button response time | < 100ms |
| NFR-003 | ROS command latency | < 200ms |
| NFR-004 | Map refresh rate | 1 Hz minimum |
| NFR-005 | Pose update rate | 1 Hz minimum |

### 4.2 Accessibility

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-101 | Minimum button size (touch) | 80x80 mm |
| NFR-102 | Font size (body text) | Minimum 16px |
| NFR-103 | Font size (headings) | Minimum 24px |
| NFR-104 | Color contrast ratio | WCAG AA (4.5:1) |
| NFR-105 | Touch target spacing | Minimum 8mm |

### 4.3 Localization

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-201 | Supported languages | Russian, English |
| NFR-202 | Language switching runtime | Without reload |
| NFR-203 | RTL support | Not required (LTR only) |
| NFR-204 | Date/time format | Localized |

### 4.4 Idle Behavior

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-301 | Idle timeout (menu screens) | 15 seconds |
| NFR-302 | Idle timeout (navigation) | Disabled |
| NFR-303 | Initial idle delay | 60 seconds |
| NFR-304 | Activity detection events | mousedown, keydown, scroll, touchstart |

### 4.5 Safety

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-401 | Emergency stop visibility | Always visible in Active Mode |
| NFR-402 | Emergency stop response | Immediate (<100ms) |
| NFR-403 | Stop on disconnect | Yes |
| NFR-404 | Watchdog timeout | Configurable |

### 4.6 Browser Support

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-501 | Target browser | Chromium-based (Chrome/Edge) |
| NFR-502 | WebSocket support | Required |
| NFR-503 | Touch support | Required |
| NFR-504 | Screen size | 10-15 inch tablet |

---

## 5. ROS2 Interface Requirements

### 5.1 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Pub | Velocity commands for manual control |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Sub | Robot position from localization |
| `/map` | `nav_msgs/OccupancyGrid` | Sub | Map data (future) |
| `/voice_activation` | `std_msgs/String` | Sub | Voice command to wake from idle |

### 5.2 Services

| Service | Type | Direction | Description |
|---------|------|-----------|-------------|
| `/save_waypoint` | `verter_admin_msgs/SaveWaypoint` | Client | Save new waypoint |
| `/delete_waypoint` | `verter_admin_msgs/DeleteWaypoint` | Client | Delete waypoint |
| `/list_waypoints` | `verter_admin_msgs/ListWaypoints` | Client | Get all waypoints |
| `/navigate_to_waypoint` | `verter_admin_msgs/NavigateToWaypoint` | Client | Start navigation |
| `/slam_start` | `std_srvs/Trigger` | Client | Start SLAM mapping |
| `/slam_stop` | `std_srvs/Trigger` | Client | Stop SLAM mapping |
| `/slam_save` | `std_srvs/Trigger` | Client | Save map |

### 5.3 Actions

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Navigation action for Nav2 |

### 5.4 Rosbridge Connection

| Parameter | Value | Description |
|-----------|-------|-------------|
| URL | `ws://localhost:9090` | Default rosbridge URL |
| Transport | WebSocket | Communication protocol |
| Reconnect | Auto | Reconnection on disconnect |
| Timeout | 5000ms | Connection timeout |

---

## 6. Scenario Flows

### 6.1 Setup: Creating New Map

```
1. Engineer opens Setup Mode (Mapping page)
2. Places robot at starting position
3. Clicks "Start Mapping"
4. Uses WASD keys to manually drive robot around area
5. Observes map being built in real-time
6. Clicks "Stop Mapping" when complete
7. Clicks "Save Map"
8. Verifies "Map saved successfully" message
```

### 6.2 Setup: Adding Waypoints

```
1. Engineer navigates to Points page
2. Clicks "Add New Location"
3. Drives robot to desired location
4. Checks "Use current position"
5. Enters location name (e.g., "Cardiology Office")
6. Clicks "Save Location"
7. Verifies location appears in list
```

### 6.3 Active: Patient Navigation

```
1. Patient sees idle screen with greeting
2. Touches screen or says "hello"
3. Main menu appears
4. Touches "Guide to Office"
5. Selects destination (e.g., "Cardiologist")
6. Confirms destination
7. Robot begins moving, "Follow me" shown
8. Patient follows robot to destination
9. "We have arrived!" screen appears
10. Screen returns to idle after 5 seconds
```

### 6.4 Active: Emergency Stop

```
1. During any active mode operation
2. User or operator presses red Emergency Stop button
3. Robot immediately stops all motion
4. Zero velocity published to /cmd_vel
5. Returns to menu screen
```

---

## 7. Validation Criteria

### 7.1 Setup Mode Validation

| Criterion | Test Method |
|-----------|-------------|
| Mapping controls work | Start/Stop/Save buttons execute without errors |
| Home point saves | Home waypoint retrievable via list service |
| Waypoint CRUD | Create, read, update, delete operations persist |
| Manual control | Robot responds to WASD commands correctly |
| Speed adjustment | Slider changes published velocity values |

### 7.2 Active Mode Validation

| Criterion | Test Method |
|-----------|-------------|
| Idle to menu transition | Touch/mouse event triggers navigation |
| Menu items navigate | All buttons route to correct pages |
| Waypoint selection | All waypoints displayed and selectable |
| Navigation flow | Full flow from select to arrive completes |
| Auto-idle works | Inactivity returns to idle screen |
| Emergency stop | Button stops robot immediately |
| Language switch | Text changes between RU/EN |

### 7.3 Integration Validation

| Criterion | Test Method |
|-----------|-------------|
| ROS connection | UI shows connected status |
| Pose updates | Current position displays live data |
| Waypoint sync | Changes in ROS reflect in UI |
| Cmd_vel publishing | Robot moves on manual control |
| Service calls | Waypoint save/delete return success |

---

## 8. Screen Map

```
/ (root)
├── /setup/
│   ├── /setup/mapping    → MappingPage
│   ├── /setup/home       → HomePage
│   └── /setup/points     → PointsPage
│
└── /active/
    ├── /active/idle      → IdlePage
    ├── /active/menu      → MenuPage
    ├── /active/select    → SelectPage
    ├── /active/confirm/:name → ConfirmPage
    ├── /active/navigation → NavigationPage
    ├── /active/arrived   → ArrivedPage
    ├── /active/find      → FindPage
    ├── /active/info      → InfoPage
    ├── /active/ask       → AskPage
    └── /active/about     → AboutPage
```

---

## 9. State Machines

### 9.1 UI Mode State

```
[Idle] → touch/key → [Menu]
[Menu] → 15s timeout → [Idle]
[Menu] → selection → [Select]
[Select] → selection → [Confirm]
[Confirm] → cancel → [Select]
[Confirm] → start → [Navigation]
[Navigation] → arrive → [Arrived]
[Arrived] → 5s → [Idle]
[Any active page] → emergency stop → [Menu]
```

### 9.2 Navigation State

```
[idle] → setGoal → [planning]
[planning] → success → [navigating]
[planning] → failure → [failed]
[navigating] → complete → [arrived]
[navigating] → cancel → [cancelled]
[navigating] → timeout → [failed]
[failed/cancelled] → reset → [idle]
[arrived] → reset → [idle]
```

---

## 10. Glossary

| Term | Definition |
|------|------------|
| Waypoint | A saved (x, y, theta) position for navigation |
| Home Point | Special waypoint for robot's charging/dock position |
| SLAM | Simultaneous Localization and Mapping |
| AMCL | Adaptive Monte Carlo Localization |
| Nav2 | ROS 2 Navigation Stack |
| Rosbridge | WebSocket bridge for ROS communication |
| Idle Screen | Attract screen shown when no user interaction |
| Active Mode | Patient-facing operational mode |
| Setup Mode | Engineer-facing configuration mode |

---

## Appendix A: Change History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-04-24 | Initial document from codebase analysis |

---

## Appendix B: References

1. ROS 2 Humble Documentation
2. Nav2 Navigation Stack
3. SLAM Toolbox
4. React Router v6
5. Zustand State Management
6. roslib.js Documentation
