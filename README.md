# Team 285C – VEX Robotics Code Repository

Welcome to the official code repository for **Team 285C**’s VEX V5 robot.  

This repository contains all the code, documentation, and resources for our robot’s autonomous routines, driver control, and subsystems for the 2026 season.

We aim to showcase not just a functioning robot, but also the **engineering process, iterative design, and teamwork** that went into creating it.

<p align="center">
  <img src="https://github.com/JaukG9/285CPushBack/blob/main/robot.webp" alt="Robot">
</p>

## Competition Results
| Location | Place | Award |
|----------|-------|-------|
|  |  |  |

## Team
| Name | Role | Responsibilities |
|------|------|-----------------|
| MarcAndre Fleury | Build Lead, Design Lead, Drive Team |  |
| Ayaan Goswami | Prog Lead, Build Team, Drive Team |  |
| Sashank Bhagavatula | Build Team, Prog Team, Drive Team |  |
| Zhe Chng | Build Team. Drive Team |  |
| Ahmar Gardezi | Build Team |  |
| Justin Kuo | Build Team |  |
| Krish Roy | Build Team |  |
| Eashan Desai | Build Team, Prog Team |  |
| Pavan Gudivada | Build Team  |  |
| Amruta Nangarla | Build Team, Prog Team |  |
| Divin Giddaluru | Build Team, Prog Team |  |
| Reddy Bommala | Build Team |  |
| Kanai Modi | Build Team |  |  |

## Key Features
### Mechanical Design
- 6-motor drivetrain with optimized 3:4 gear ratio for speed and control
- Angled front drivetrain to guide blocks into intake
- Reinforced structure with cross-bracing to prevent bending
- Extended sled system for smooth parking

### Scoring Mechanisms
- Multi-level scoring: long goal and middle goal
- Dual descoring systems: bunny ears and RAM
- High-speed intake and conveyor system
- Match loader scraper for rapid block collection

### Programming & Autonomy
- Odometry system using IMU and tracking wheel
- PID-controlled autonomous movement
- Multiple autonomous routines (skills + match)
- Automatic anti-jamming system based on velocity detection

### Reliability Improvements
- Dual-chain intake for redundancy
- Anti-jam funnel in conveyor
- Structural redesign to reduce torsion
- PTO system for efficient motor usage

### Innovation
- Integrated PTO system controlling multiple subsystems
- Extendable mechanisms within size constraints
- Autonomous parking optimization
- Iterative design improvements based on competition performance

## Tech Stack
- PROS (C++)
- LemLib (odometry)
- LVGL (GUI)
- fmt (formatting)

## Detailed Breakdown

### Autonomous Routines
| Routine       | Purpose                                                                                   |
|---------------|-------------------------------------------------------------------------------------------|
| skip          | Moves forward 6 inches to avoid interfering with solo AWP autonomous                        |
| left_4Rush    | Collects 3 left-side blocks + preload and scores 4 in left long goal, then wings control   |
| right_4Rush   | Collects 3 right-side blocks + preload and scores 4 in right long goal, then wings control |
| left_7Rush    | Collects 3 left-side blocks + preload, scores in long goal, then wings blocks              |
| right_7Rush   | Collects 3 right-side blocks + preload, scores in long goal, then wings blocks             |
| left_43Split  | Collects 3 left-side blocks, scores mid goal, then scores 4 in long goal and wings control |
| right_43Split | Collects 3 right-side blocks, scores mid goal, then scores 4 in long goal and wings control|
| awp           | Solo AWP autonomous (left side)                                                            |
| skills        | Full programming skills autonomous: collects and scores across the field, ends parked      |


### Driver Control Mapping
| Controller | Function |
|------------|----------|
| R2 | Activate PTO and run conveyor forward
| L2 | Run conveyor forward (without PTO)
| L1 | Run conveyor backward
| A | Toggle trapdoor piston
| X | Toggle scraper piston
| R1 | Toggle wing piston
| Left/Right Joysticks | Drive robot with tank controls

## File Structure:
```plaintext
285CPushBack/
│
├── firmware/
│
├── include/
│   ├── fmt/              # external (don’t touch)
│   ├── lemlib/           # external (don’t touch)
│   ├── liblvgl/          # external (don’t touch)
│   ├── pros/             # external (don’t touch)
│   │
│   ├── robot/            # OUR HEADERS
│   │   ├── autos.h
│   │   ├── drivetrain.h
│   │   ├── functions.h
│   │   ├── odometry.h
│   │   └── robot-config.h
│   │
│   ├── api.h             # PROS core
│   └── main.h
│
├── src/
│   ├── autos.cpp
│   ├── drivetrain.cpp
│   ├── functions.cpp
│   ├── main.cpp
│   ├── odometry.cpp
│   └── robot-config.cpp
│
├── Makefile
├── common.mk
├── project.pros
├── README.md
└── .gitignore
```

You can view our Engineering Notebook here: [Engineering Notebook]()

