# IDC2026 Team Orange — Cyber Game

This repository contains Team Orange's IDC2026 Cyber Game participant files,
the complete runnable package, and the supporting delivery documentation.

## Cyber Game overview

The Cyber Game is the MuJoCo-based Cyber World competition. The organizer
provides the Go2 quadruped, field, timing, randomized target configuration,
scoring, and referee logic. The participant side provides:

- `user_side_go2.py`: the Go2 controller and actuator command state machine;
- `user_mechanism.xml`: the participant mechanism attached to the Go2 model.

During a match, the official field program passes the public observation to
the participant controller. The controller returns named actuator commands;
the official field program applies those commands and advances MuJoCo physics.
The participant files are not standalone programs.

## Repository contents

```text
.
├── README.md
├── submission/
│   ├── user_side_go2.py
│   └── user_mechanism.xml
├── packages/
│   ├── IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip
│   ├── IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip
│   └── IDC2026_Team_Orange_Cyber_Game_Submission_Bundle.zip
└── docs/
    ├── CYBER_GAME_COMPLETE_PACKAGE_INFO.txt
    ├── CYBER_GAME_MINIMAL_PACKAGE_INFO.txt
    └── IDC2026_Team_Orange_Cyber_Game_Submission_Package_INFO.txt
```

## Which package should be used?

### Complete package

Use `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip` when a
complete Cyber Game runtime package is requested. It includes the provided
0814 official runtime, scene, MuJoCo model, assets, current Team Orange
participant files, and a PowerShell launcher. Python and third-party package
installation are still required on the host computer.

### Minimal package

Use `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip` when the
organizer requests only the participant-authored files. Copy the two files
from its `cyber_submission` folder into the organizer's official Cyber Game
directory.

### Bundle package

`packages/IDC2026_Team_Orange_Cyber_Game_Submission_Bundle.zip` is an outer
delivery bundle containing both ZIP packages and a short English explanation.

## Environment requirements

The organizer's stated evaluation environment is:

- Windows 11;
- Python 3.13.9;
- MuJoCo Python package, including `mujoco.viewer` for the graphical runner;
- NumPy.

ONNX Runtime is not required. The participant controller does not import or
use ONNX Runtime, PyTorch, OpenCV, ROS, Gazebo, a camera, or an external model
file. The ZIP packages do not include a Python interpreter or third-party
Python packages; those must be installed or provided by the evaluation
environment.

## PowerShell run instructions

The complete package is prepared for PowerShell. Do not use the Command Prompt
syntax `set IDC2026_SIDE=blue` in PowerShell.

Extract the complete package, open PowerShell in the extracted
`IDC2026_CyberGame` directory, and run:

```powershell
.\run_match.ps1 -Side red
.\run_match.ps1 -Side blue
```

The launcher sets `IDC2026_SIDE`, changes to the correct runtime directory,
and starts the official `run_match.py` program. If the minimal package is
being used with an organizer-provided runtime, run it manually in PowerShell:

```powershell
$env:IDC2026_SIDE = "blue"
python .\run_match.py
```

## Verification status

The current participant files have passed the local interface and XML test
suite (`12/12`). Red and blue preflight checks pass, including the mechanism
actuator interface, initial state, Start B envelope, and model loading. The
complete ZIP has also been checked for archive integrity and extracted-package
loading. These checks were performed in the development environment and do
not replace the organizer's final Windows evaluation.

## Submission boundary

The official participant boundary is the two files under `submission/`. The
field simulator, scoring, target randomization, timing, and physics stepping
remain organizer-side responsibilities. The complete ZIP is provided for
reproducibility and local evaluation; the organizer's newer official runtime,
if supplied, should take precedence for the final contest run.
