# 285C Discobots Chaos — VEX V5 Push Back

**Carnegie Vanguard High School · Houston, TX · 2025–2026 Season**

[![Robot](https://github.com/JaukG9/285CPushBack/raw/main/robot.jpg)](https://github.com/JaukG9/285CPushBack/blob/main/robot.webp)

> **Worlds Qualifiers:** 285C qualified for the 2026 VEX Robotics World Championship, the first time a CVHS Robotics team has done so in over five years. All three 285 sister teams qualified together.

---

## Table of Contents

- [About the Team](#about-the-team)
- [Competition Results](#competition-results)
- [Robot Design](#robot-design)
- [Tech Stack](#tech-stack)
- [Code Architecture](#code-architecture)
  - [File Structure](#file-structure)
  - [Custom Chassis & Motion Chaining](#custom-chassis--motion-chaining)
  - [Odometry & PID](#odometry--pid)
  - [Autonomous Routines](#autonomous-routines)
  - [Anti-Jamming System](#anti-jamming-system)
  - [Autonomous Parking](#autonomous-parking)
  - [Auton Selector](#auton-selector)
  - [Driver Control](#driver-control)
  - [PTO System](#pto-system)
- [Engineering Notebook](#engineering-notebook)

---

## About the Team

285C — **Discobots Chaos** — is a VEX V5 Robotics team from Carnegie Vanguard High School in Houston, Texas. For most members, this was their first full competitive robotics season.

| Name | Role | Years of Robotics |
|---|---|---|
| Marc-Andre Fleury | Build Lead, Design Lead, Drive Team | 2 |
| Ayaan Goswami | Prog Lead, Build Team, Drive Team | 3 |
| Zhe Chng | Build Team, Drive Team | 3 |
| Krish Roy | Build Team | 3 |
| Eashan Desai | Build Team, Prog Team | 1 |
| Amruta Nangarla | Notebook Lead, Build Team, Prog Team | 3 |

---

## Competition Results

| Competition | Date | Rank | Highest Elimination Round |
|---|---|---|---|
| Katy Holiday Classic V5RC Push Back | Nov. 11, 2025 | 34th | DNQ |
| Den Wars: The Push Back Battle | Feb. 7, 2026 | 4th | Semifinals |
| NEW YEAR NEW DRIVE (GORTS) 2026 | Feb. 21, 2026 | 22nd | Semifinals |
| Texas V5RC High School Region 3 Championship | Feb. 26, 2026 | 6th | Quarterfinals |

### Skills Progression

| Competition | Auton Skills | Driver Skills | Combined |
|---|---|---|---|
| Den Wars | 25 pts | 75 pts | 100 pts |
| GORTS | 40 pts | 62 pts | 102 pts |
| Texas Region 3 (States) | 53 pts | 84 pts | **137 pts (9th)** |

At States, we placed **6th in qualifications** (7W–2L, 15 WP, **80 AP** - highest AP in the competition), chose 9204B Dragons as their alliance, won the Round of 16, and lost in Quarterfinals. They ultimately qualified for Worlds through a **skills rolldown**, making it the first Worlds qualification for a CVHS Robotics team in over five years.

---

## Robot Design

### Drivetrain

- 6-motor, 6-wheel tank drive with a 3:4 gear ratio (600 RPM motors → 450 RPM output)
- 3.25" omni wheels on the outside, traction wheels in the center for lateral stability
- 11.22" track width
- After Regionals, the front of each drivetrain module was redesigned from flat to angled/slanted, which guides blocks directly into the intake path rather than deflecting them away
- Cross-bracing added after Den Wars when a collision bent the frame and stopped one side from moving

### Intake & Conveyor

- 2-motor high-speed intake using flex wheels and sprockets on a dual-chain configuration (redundance in case primary chain disengages)
- Conveyor transports blocks from intake to the top/middle extake; includes a polycarbonate funnel to prevent jamming
- Post-States rebuild: conveyor redesigned into a single unified rectangular frame with standoff cross-bracing to prevent torsion, with motor-powered axles supported on both ends

### Scoring Mechanisms

- Long goal extake: rubber-band track platform with banded gears and flex wheels; angled funnel ensures blocks land in the control zone
- Middle extake (midtake): piston pulls down conveyor floor to redirect blocks horizontally; shares a motor with the long goal extake via gears. Includes a polycarbonate funnel
- Scraper (match loader): piston-actuated mechanism at the front of the bot for scraping blocks from the field loader
- Long goal aligner: thick lexan triangle mounted to the rear of the bot for precise goal alignment

### Descoring

- Wing / Bunny Ear: pneumatically extended wing mechanism; fits within 18×18×18" at rest and extends during play to sweep blocks out of the opponent's long goal
- Ram: polycarbonate plate on standoffs used for descoring when the wing can't reach, and for pushing blocks into controlled zones

### Other

- Odometry tracking wheel: vertically tracking wheel attached on a rotation sensor
- Front sleds: extended ramps on both sides of the bot's undercarriage for smooth entry into the parking zone

---

## Tech Stack

| Tool | Purpose |
|---|---|
| [PROS](https://pros.cs.purdue.edu/) (C++) | Primary robot programming framework |
| [LemLib](https://lemlib.readthedocs.io/) | Odometry, PID motion control |
| [LVGL](https://lvgl.io/) | On-brain GUI (autonomous selector display) |
| [fmt](https://fmt.dev/) | String formatting |

---

## Code Architecture

### File Structure

```
285CPushBack/
│
├── firmware/
│
├── include/
│   ├── fmt/              # External — do not modify
│   ├── lemlib/           # External — do not modify
│   ├── liblvgl/          # External — do not modify
│   ├── pros/             # External — do not modify
│   │
│   └── robot/            # Team headers
│       ├── autos.h
│       ├── drivetrain.h
│       ├── functions.h
│       ├── odometry.h
│       └── robot-config.h
│
├── src/
│   ├── autos.cpp         # All autonomous routines (7 match + 1 skills)
│   ├── drivetrain.cpp    # Drivetrain initialization
│   ├── functions.cpp     # Conveyor movement, pneumatic toggling, helpers
│   ├── main.cpp          # Entry point — init, autonomous dispatch, opcontrol
│   ├── odometry.cpp      # Positional tracking, PID setup, CustomChassis definition
│   └── robot-config.cpp  # Motors, pistons, global variables
│
├── Makefile
├── common.mk
├── project.pros
└── README.md
```

### Module Descriptions

| File | Description |
|---|---|
| `robot-config.cpp` | Initializes the `pros::Controller`, the `pros::MotorGroup` for the conveyor (`conveyor`), and `pros::adi::DigitalOut` objects for each piston. Also defines global boolean flags tracking each piston's current extended/retracted state, shared across all files. |
| `drivetrain.cpp` | Initializes the LemLib drivetrain object with wheel diameter, track width, gear ratio, and horizontal drift constant (`2`). |
| `odometry.cpp` | Defines the `lemlib::OdomSensors` object, lateral/angular PID controllers, steering curve, and the `CustomChassis` object. |
| `functions.cpp` | Contains `runConveyor(int rpm)` and `ptoChange()` for conveyor and PTO control, as well as pneumatic toggle helpers. |
| `autos.cpp` | All autonomous route implementations. Called from `main.cpp` via a switch-case selector. |
| `main.cpp` | Handles initialization (chassis init, brain screen, pistons to default state), autonomous dispatch (switch on selected route), and the driver control loop. |

---

### Custom Chassis & Motion Chaining

One of the most significant programming innovations this season is the `CustomChassis` class, defined in `odometry.h`. It extends LemLib's built-in `lemlib::Chassis`, inheriting all of its methods and attributes, while adding two new methods that automatically compute motion chaining parameters at runtime.

Without `CustomChassis`, chaining sequential motions in LemLib requires manually tuning `earlyExitRange` and `minSpeed` for each individual movement. `CustomChassis` eliminates this by deriving both values from kinematic first principles, based on the actual distance of each motion.

#### `chainToPoint(x, y, heading, speed, ...)`

Wraps `lemlib::Chassis::moveToPoint()` with automatic chaining parameters.

**Algorithm:**
1. Computes Euclidean distance `d` from current pose `(x₀, y₀)` to target `(x, y)`
2. Calculates the maximum achievable speed over that distance using the UAM velocity formula:
   ```
   v_peak = sqrt(2 * a * d)
          = sqrt(1000 * d)    // after unit conversion
   ```
3. Sets `minSpeed = v_peak / 2` — the speed the robot must maintain at the exit point to carry momentum into the next motion
4. Sets `earlyExitRange = 0.15 * d * (speed / 127.0)` — how early the robot begins transitioning out, scaled by both distance and commanded speed percentage
5. Calls `moveToPoint()` with these computed parameters

#### `chainToHeading(theta, speed, ...)`

Wraps `lemlib::Chassis::turnToHeading()` with automatic chaining parameters.

**Algorithm:**
1. Computes angular distance `d` (in degrees) between current heading and target heading `theta`
2. Calculates peak angular speed:
   ```
   v_peak = sqrt(2 * a * d)
          = sqrt(500 * d)     // after unit conversion for angular motion
   ```
3. Sets `minSpeed = v_peak / 2`
4. Sets `earlyExitRange = 0.08 * d * (speed / 127.0)` — tighter than the lateral version (8% vs 15%) since turns require more precision to avoid overshoot
5. Calls `turnToHeading()` with these computed parameters

These methods are used throughout `autos.cpp` whenever sequential motion continuity is desired — the robot flows smoothly from point to point without stopping.

---

### Odometry & PID

**Sensors:**
- IMU (port 13): tracks linear acceleration and angular velocity; primary source of heading and position
- Vertical tracking wheel (port 12): supplements IMU to reduce accumulated lateral drift. Traction wheels eliminate most sideways slip, but the tracking wheel corrects residual error

LemLib's `lemlib::OdomSensors` is initialized with the vertical tracking wheel and IMU; horizontal tracking wheel slots are passed `nullptr`.

**Drivetrain parameters:**
- Track width: `11.22"`
- Wheel diameter: `3.25"` (omni)
- RPM: `450` (after 3:4 gear reduction)
- Horizontal drift constant: `2`

**PID Tuning:**

| Controller | Gains Used | Notes |
|---|---|---|
| Lateral | kP, kD | kI set to 0 — no persistent offset warranting integral correction |
| Angular | kP, kD | kI set to 0 for the same reason |

Additional tuned parameters:
- Small/large error range: 1" and 3" — prevents infinite oscillation near the target
- Small/large error timeout: 100ms and 500ms
- Lateral slew: enabled to limit acceleration and prevent tipping (center of gravity is relatively high); angular slew disabled

**Steering Curve:**
- Deadband: `10/127` — ignores joystick values below this threshold to prevent drift and micromovement
- Minimum output: `10/127` — motors only activate if they would supply at least this much power
- Exponential gain: `1.019` — allows finer low-speed control while still reaching full speed

---

### Autonomous Routines

The autonomous selector in `main.cpp` uses a `switch` statement to dispatch the correct routine based on the selection made on the brain screen via LVGL.

| Routine | Side | Description |
|---|---|---|
| `skip` | — | Moves forward 6" to avoid interfering with a partner's solo AWP |
| `awp` | Left | Solo AWP autonomous routine |
| `left_4Rush` | Left | Collects 3 left-side blocks + preload → scores 4 in left long goal → wings for zone control |
| `right_4Rush` | Right | Mirror of `left_4Rush` on the right side |
| `left_7Rush` | Left | Collects 3 left-side blocks + preload → scores in long goal → wings additional blocks |
| `right_7Rush` | Right | Mirror of `left_7Rush` |
| `left_43Split` | Left | Collects 3 left blocks → scores in middle goal → scores 4 in long goal → wings |
| `right_43Split` | Right | Mirror of `left_43Split` |
| `skills` | — | Full 60-second programming skills routine (see below) |

#### Skills Autonomous Detail

The skills routine starts at `(-48, -14.2, 180°)` and proceeds as follows:

1. Deploys scraper → drives to `(-48, -48)` → turns west and sweeps left wall to `(-120, -48)` while running intake for 2.5 seconds
2. Retracts scraper → activates bunny ear → resets pose to `(-57, -48, 270°)` → repositions to `(-36, -57)` and `(-36, -62)`
3. Crosses field to `(46, -61)` → resets pose → performs slow scoring "crawl" from `(25, -48)` to `(10, -48)` running intake + extake simultaneously
4. Re-deploys scraper → sweeps right wall to `(120, -48)` → repeats slow-crawl scoring pass
5. Transitions to `(41, 50.67)` → resets pose → repeats wall sweep and scoring crawl on the opposite side of the field
6. Resets to `(-25, 48, 270°)` → moves to `(-48, 48)` → aligns to `(-64, 18, 90°)` → activates `odomLift` → re-deploys scraper → drives toward `(-64, -5)` for **10 seconds while running intake and extake to park**

---

### Anti-Jamming System

During Den Wars, the one-motor intake lacked sufficient torque to push blocks through to the upper extake at certain points, causing jams mid-auton. The fix: before outtaking into the long goal, the code checks whether the conveyor is moving at **less than 50/127 of its maximum RPM** using LemLib's `.get_actual_velocity()`. If it is, the conveyor reverses for **500ms** to unjam, then resumes normal outtake.

```cpp
// Pseudo-representation of anti-jam logic
if (conveyor.get_actual_velocity() < 0.5 * MAX_RPM) {
    runConveyor(-MAX_RPM);   // reverse to unjam
    pros::delay(500);
}
runConveyor(MAX_RPM);        // resume outtake
```

---

### Autonomous Parking

Parking in skills yields **20 points** for clearing and occupying the alliance park zone. The routine implements parking in three steps:

1. **Position**: Move the robot alongside the alliance wall facing the park zone using `moveToPoint()` and `moveToPose()`
2. **Enter**: Extend the scraper to clear any blocks in the park zone; while `robot.y > -10` (outside the zone), drive toward a point past that y-coordinate at maximum speed
3. **Correct**: Move slightly backwards to settle position; run extake to clear any remaining blocks from the conveyor so they don't cause a jam on restart

---

### Auton Selector

The brain screen is initialized in `main.cpp` using LVGL. The selector allows the drive team to choose the appropriate autonomous routine before a match by interacting with the screen. The selected routine index is stored and dispatched via a `switch` statement in the `autonomous()` function.

---

### Driver Control

**Tank Drive** — left joystick controls left motor group, right joystick controls right motor group.

| Input | Action |
|---|---|
| Left Joystick | Left drivetrain motors |
| Right Joystick | Right drivetrain motors |
| D-Pad Up | Drive forward at full speed (127V) |
| D-Pad Down | Drive backward at full speed (-127V) |
| R2 | Activate PTO + run conveyor forward (outtake mode) |
| L2 | Run conveyor forward (intake, no PTO) |
| L1 | Run conveyor backward (low-goal funneling) |
| R1 | Toggle wing piston |
| A | Toggle trapdoor piston |
| X | Toggle scraper piston |

---

### PTO System

Both non-drivetrain motors run all three subsystems: intake, conveyor, and top extake. To prevent blocks from being outtaked unintentionally during intake, a **Power Take-Off (PTO)** piston mechanically decouples the outtake. 

In code, `ptoChange()` in `functions.cpp` toggles the global `ptoActivated` boolean and flips the PTO piston state. This is called when switching between intake and outtake modes during both driver control and autonomous routines.

---

## Engineering Notebook

Our full engineering notebook documents the design process, build iterations, competition reviews, and programming decisions for the entire season.

[View Engineering Notebook](https://docs.google.com/presentation/d/1lr190siZyLemRTPEoyDQzBPVKPbAYJWm4jTHMkNOKCQ/edit?usp=sharing)

The notebook covers:
- Full team bios and role assignments
- Game rules analysis and how they shaped design decisions
- Iterative build documentation (drivetrain, intake, conveyor, extake, descoring, odometry, sleds)
- Detailed programming write-ups for every major system
- Competition reviews for Katy Holiday Classic, Den Wars, GORTS, and Texas Region 3 Championship
- Worlds qualification and rebuild planning

---

*285C Discobots Chaos · Carnegie Vanguard High School · Houston, TX*