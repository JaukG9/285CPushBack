# Team 285C – VEX Robotics Code Repository

Welcome to the official code repository for **Discobots Chaos (285C)**’s VEX V5 robot.  

This repository contains all the code, documentation, and resources for our robot’s autonomous routines, driver control, and subsystems for the 2026 season.

We aim to showcase not just a functioning robot, but also the **engineering process, iterative design, and teamwork** that went into creating it.

<p align="center">
  <img src="https://github.com/JaukG9/285CPushBack/blob/main/robot.webp" alt="Robot">
</p>

## Competition Results
| Competition | Date | Place | Highest Elimination Round |
|----------|-------|:-----:|-------|
| Texas V5RC High School Region 3 Championship | Feb. 26, 2026 | 6th | Quarterfinals |
| NEW YEAR NEW DRIVE 2026 | Feb. 21, 2026 | 22nd | Semifinals |
| Den Wars: The Push Back Battle | Feb. 7, 2026 | 4th | Semifinals |
| Katy Holiday Classic V5RC Push Back [Blended] | Nov. 11, 2025 | 34th | DNQ |

## Team
| Name | Role | Years of Robotics |
|------|------|:---------------:|
| Marc-Andre Fleury | Build Lead, Design Lead, Drive Team | 3 |
| Ayaan Goswami | Prog Lead, Build Team, Drive Team | 3 |
| Sashank Bhagavatula | Build Team, Prog Team, Drive Team | 4 |
| Zhe Chng | Build Team. Drive Team | 3 |
| Ahmar Gardezi | Build Team | 1 |
| Justin Kuo | Build Team | 4 |
| Krish Roy | Build Team | 3 |
| Eashan Desai | Build Team, Prog Team | 1 |
| Pavan Gudivada | Build Team  | 1 |
| Amruta Nangarla | Notebook Lead, Build Team, Prog Team | 3 |
| Divin Giddaluru | Build Team, Prog Team | 2 |
| Reddy Bommala | Build Team | 1 |
| Kanai Modi | Build Team | 1 |

## Key Features
### Mechanical Design
- 6-motor drivetrain with optimized 3:4 gear ratio for torque and control
- Scraper for matchloading
- Angled front drivetrain to guide blocks into intake
- Reinforced structure with cross-bracing to prevent bending
- Extended sled system for smooth parking

### Scoring Mechanisms
- Multi-level scoring: long goal + upper & lower middle goals
- Dual descoring systems: wing and ram descore
- 2-motor high-speed intake and conveyor system

### Programming & Autonomy
- Odometry system using IMU and tracking wheel
- PID-controlled autonomous movement
- Multiple autonomous routines (skills + match)
- Automatic anti-jamming system based on motor velocity detection
- Custom Chassis object including automatic motion chaining methods

### Reliability Improvements
- Dual-chain intake for redundancy
- Scoring funnel in conveyor
- Structural redesign to reduce torsion
- PTO system to separate intake from outtake

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
| Routine       | Purpose                                                                                    |
|---------------|--------------------------------------------------------------------------------------------|
| skip          | Moves forward 6 inches to avoid interfering with solo AWP autonomous                       |
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
| L1 | Run conveyor backward (with low goal funneling)
| R1 | Toggle wing piston
| A | Toggle trapdoor piston
| X | Toggle scraper piston
| Left/Right Joysticks | Drive robot with tank controls
| Up/Down Arrows | Drive robot straight forwards and back

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

You can view our Engineering Notebook here: [Engineering Notebook](https://docs.google.com/presentation/d/1lr190siZyLemRTPEoyDQzBPVKPbAYJWm4jTHMkNOKCQ/edit?usp=sharing)
