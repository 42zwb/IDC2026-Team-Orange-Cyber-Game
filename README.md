# IDC2026 Team Orange — Cyber Game

This is a bilingual README. The English documentation is provided first,
followed by the Chinese documentation.

## English

### Overview

This repository contains Team Orange's IDC2026 Cyber Game participant files,
the complete runnable package, and the supporting delivery documentation.

The Cyber Game is the MuJoCo-based Cyber World competition. The organizer
provides the Go2 quadruped, field, timing, randomized target configuration,
scoring, and referee logic. The participant side provides:

- `user_side_go2.py`: the Go2 controller and actuator command state machine;
- `user_mechanism.xml`: the participant mechanism attached to the Go2 model.

During a match, the official field program passes the public observation to
the participant controller. The controller returns named actuator commands;
the official field program applies those commands and advances MuJoCo physics.
The participant files are not standalone programs.

### Repository contents

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

### Package selection

#### Complete package

Use `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip` when a
complete Cyber Game runtime package is requested. It includes the provided
0814 official runtime, scene, MuJoCo model, assets, current Team Orange
participant files, and a PowerShell launcher. Python and third-party package
installation are still required on the host computer.

#### Minimal package

Use `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip` when the
organizer requests only the participant-authored files. Copy the two files
from its `cyber_submission` folder into the organizer's official Cyber Game
directory.

#### Bundle package

`packages/IDC2026_Team_Orange_Cyber_Game_Submission_Bundle.zip` is an outer
delivery bundle containing both ZIP packages and a short English explanation.

### Environment requirements

The organizer's stated evaluation environment is:

- Windows 11;
- Python 3.13.9;
- the MuJoCo Python package, including `mujoco.viewer` for the graphical runner;
- NumPy.

ONNX Runtime is not required. The participant controller does not import or
use ONNX Runtime, PyTorch, OpenCV, ROS, Gazebo, a camera, or an external model
file. The ZIP packages do not include a Python interpreter or third-party
Python packages; those must be installed or provided by the evaluation
environment.

### PowerShell run instructions

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

### Verification status

The current participant files have passed the local interface and XML test
suite (`12/12`). Red and blue preflight checks pass, including the mechanism
actuator interface, initial state, Start B envelope, and model loading. The
complete ZIP has also been checked for archive integrity and extracted-package
loading. These checks were performed in the development environment and do
not replace the organizer's final Windows evaluation.

### Submission boundary

The official participant boundary is the two files under `submission/`. The
field simulator, scoring, target randomization, timing, and physics stepping
remain organizer-side responsibilities. The complete ZIP is provided for
reproducibility and local evaluation; the organizer's newer official runtime,
if supplied, should take precedence for the final contest run.

---

## 中文

### 项目概览

本仓库包含 Team Orange 的 IDC2026 Cyber Game 参赛文件、完整可运行包以及
相关交付说明文档。

Cyber Game 是基于 MuJoCo 的 Cyber World 比赛。主办方负责提供 Go2 四足机器狗、
比赛场地、计时、随机目标配置、计分和裁判逻辑。选手侧提供以下两个文件：

- `user_side_go2.py`：Go2 控制器和执行器命令状态机；
- `user_mechanism.xml`：挂载到 Go2 模型上的选手机构。

比赛运行时，官方场地程序把公开 observation 传给选手控制器；控制器返回带名称的
执行器命令，由官方场地程序应用命令并推进 MuJoCo 物理仿真。两个选手文件不能单独
启动比赛。

### 仓库内容

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

### 压缩包选择

#### 完整包

使用 `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip` 可获得完整的
Cyber Game 运行包。它包含 0814 官方运行环境、场景、MuJoCo 模型、资源文件、当前
Team Orange 参赛文件和 PowerShell 启动脚本，但电脑仍需安装 Python 和第三方依赖。

#### 简略包

使用 `packages/IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip` 可获得仅包含
选手编写文件的提交包。若主办方要求只提交选手文件，应把其中两个文件从
`cyber_submission` 文件夹复制到官方 Cyber Game 运行目录。

#### 合集包

`packages/IDC2026_Team_Orange_Cyber_Game_Submission_Bundle.zip` 是外层交付包，包含
完整包、简略包以及介绍两个压缩包区别的英文说明。

### 环境要求

主办方说明的评测环境为：

- Windows 11；
- Python 3.13.9；
- MuJoCo Python 包，包括图形运行所需的 `mujoco.viewer`；
- NumPy。

不需要安装 ONNX Runtime。选手控制器不使用 ONNX Runtime、PyTorch、OpenCV、ROS、
Gazebo、摄像头或外部模型文件。压缩包不包含 Python 解释器和第三方 Python 包，
这些依赖需要由评测电脑预先安装或由主办方环境提供。

### PowerShell 运行方式

完整包按 PowerShell 准备。PowerShell 中不要使用 CMD 的
`set IDC2026_SIDE=blue` 写法。

解压完整包后，在包含 `run_match.ps1` 的 `IDC2026_CyberGame` 目录中打开 PowerShell，
运行：

```powershell
.\run_match.ps1 -Side red
.\run_match.ps1 -Side blue
```

启动脚本会设置 `IDC2026_SIDE`，切换到正确的运行目录，并启动官方 `run_match.py`。
使用简略包配合主办方官方运行环境时，可手动运行：

```powershell
$env:IDC2026_SIDE = "blue"
python .\run_match.py
```

### 验证状态

当前选手文件已通过接口和 XML 测试，结果为 `12/12`。红蓝双方预检均通过，包括机构
执行器接口、初始状态、Start B 包络和模型加载。完整 ZIP 也已通过压缩包完整性和
解压后的加载检查。这些检查在开发环境完成，不能替代主办方最终的 Windows 评测。

### 提交边界

官方选手提交边界是 `submission/` 下的两个文件。场地仿真、计分、目标随机化、计时
和物理推进仍由主办方程序负责。完整 ZIP 用于复现和本地评测；如果主办方提供更新的
官方运行包，正式比赛应优先使用主办方的版本。
