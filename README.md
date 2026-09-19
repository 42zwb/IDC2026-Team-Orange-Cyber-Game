# IDC2026 Team Orange — Cyber Game

This repository is an annotated archive of Team Orange's IDC2026 Cyber Game
work. It separates the contest rules, organizer-provided baseline files,
participant-authored strategy and mechanism files, and packages for submission
or local reproduction.

## English

### What this repository is for

The repository is not only a code drop. Its main purpose is to show which
parts define the contest and runtime, and which parts are Team Orange's own
work.

The Cyber Game is a MuJoCo-based Cyber World competition. The organizer-side
runtime provides the Go2 model, field, timing, randomized target setup,
scoring, referee logic, and the participant programming interface. Team Orange
provides the control strategy and participant mechanism that run through that
interface.

### Ownership map

| Category | Owner | Repository location | Meaning |
| --- | --- | --- | --- |
| Rulebook | Organizer | `rulebook/` | Official contest rules and constraints |
| Runtime baseline | Organizer | `official/` | 0814 field program, model, scene, assets, and sample interface |
| Participant work | Team Orange | `submission/source/` | Current controller strategy and mechanism design |
| Minimal delivery | Team Orange | `submission/*.zip` | The participant-authored files prepared for submission |
| Complete local package | Team Orange integration package | `runtime/` | Organizer baseline combined with the current participant files |

### Repository layout

```text
.
├── README.md
├── rulebook/
│   └── Contest_Rules_final2026.pdf
├── official/
│   ├── ORGANIZER_FILES_INFO.txt
│   ├── IDC2026_CyberGame_Organizer_Complete_0814.zip
│   └── IDC2026_CyberGame_Organizer_Minimal_0814.zip
├── submission/
│   ├── source/
│   │   ├── user_side_go2.py
│   │   └── user_mechanism.xml
│   ├── CYBER_GAME_MINIMAL_PACKAGE_INFO.txt
│   └── IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip
└── runtime/
    ├── CYBER_GAME_COMPLETE_PACKAGE_INFO.txt
    └── IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip
```

### Organizer-provided materials

#### Rulebook

`rulebook/Contest_Rules_final2026.pdf` is the IDC2026 Contest Rule Book,
Version 1.01 dated 5 August 2026. It is the reference for the contest rules;
implementation details in the local packages do not override it.

#### Complete organizer baseline

`official/IDC2026_CyberGame_Organizer_Complete_0814.zip` is the complete 0814
baseline associated with the organizer's Cyber Game package. It contains the
organizer-side field program, `field_side.py`, `run_match.py`, the MuJoCo
scene, Go2 model, humanoid models, assets, and the sample participant files.

The organizer-side files own the field, physics stepping, match timer,
randomized goals, scoring, and referee logic. They are included so visitors
can understand the environment in which the participant code runs.

#### Minimal organizer interface

`official/IDC2026_CyberGame_Organizer_Minimal_0814.zip` is a compact,
unmodified copy of the participant-facing files from the same 0814 baseline:

- the official sample `user_side_go2.py` participant program;
- the official sample `user_mechanism.xml` mechanism;
- the official package instructions.

This archive is for interface comparison. It is not Team Orange's final
submission and does not contain the complete field runtime.

### Team Orange participant work

The current participant-authored files are available directly under
`submission/source/`:

- `user_side_go2.py`: the Go2 control strategy, observations, actuator
  commands, side selection, walking behavior, and ball-handling logic;
- `user_mechanism.xml`: the participant-designed mechanism attached to the
  Go2 model.

These files are the part that represents Team Orange's strategy and mechanism.
They are not standalone programs: the organizer-side runtime imports them,
passes observations to the controller, applies returned actuator commands, and
advances MuJoCo physics.

### Minimal submission package

Use `submission/IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip` when
only the participant-authored files are requested. It contains the current
`user_side_go2.py`, `user_mechanism.xml`, and the package instructions. It does
not contain the organizer's field simulator, scene, model, or assets.

### Complete local runtime package

Use `runtime/IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip` for local
reproduction. It combines the 0814 runtime snapshot with the current Team
Orange participant files and a PowerShell launcher. This is a Team Orange
integration package for reproducibility; it is not the original organizer
baseline and should not replace a newer official runtime during final judging.

The stated evaluation environment is:

- Windows 11;
- Python 3.13.9;
- the MuJoCo Python package, including `mujoco.viewer` for the graphical runner;
- NumPy.

ONNX Runtime is not required. The packages do not include Python or third-party
Python packages; those must be installed or provided by the evaluation
environment.

### Running the complete package

The complete package is prepared for PowerShell. After extraction, open
PowerShell in the extracted `IDC2026_CyberGame` directory and run:

```powershell
.\run_match.ps1 -Side red
.\run_match.ps1 -Side blue
```

For manual execution, use PowerShell environment-variable syntax:

```powershell
$env:IDC2026_SIDE = "blue"
python .\run_match.py
```

Do not use the Command Prompt syntax `set IDC2026_SIDE=blue` in PowerShell.

### Validation boundary

The current participant files passed the local interface and XML test suite
(`12/12`). Red and blue preflight checks also passed, including the mechanism
actuator interface, initial state, Start B envelope, and model loading. ZIP
integrity and extracted-package loading were checked as well. These are local
development checks and do not replace the organizer's final evaluation.

The practical boundary is therefore:

- read `rulebook/` for the contest rules;
- inspect `official/` to understand the supplied problem and runtime;
- inspect `submission/source/` to see Team Orange's strategy and mechanism;
- use `submission/*.zip` for participant-only delivery;
- use `runtime/*.zip` for local reproduction with the integrated package.

---

## 中文

### 仓库用途

本仓库是 Team Orange 的 IDC2026 Cyber Game 资料和代码归档。仓库将比赛规则、
主办方下发的环境文件、选手自行编写的策略与机构文件，以及提交包和本地运行包
分开保存，方便访问者判断每个文件的来源和作用。

Cyber Game 是基于 MuJoCo 的 Cyber World 比赛。主办方运行环境负责提供 Go2 模型、
比赛场地、计时、随机目标配置、计分、裁判逻辑和选手编程接口。Team Orange 负责
提供通过该接口运行的控制策略和选手机构设计。

### 文件归属

| 类别 | 归属 | 仓库位置 | 含义 |
| --- | --- | --- | --- |
| 比赛规则 | 主办方 | `rulebook/` | 正式比赛规则和限制 |
| 运行环境基线 | 主办方 | `official/` | 0814 场地程序、模型、场景、资源和示例接口 |
| 选手代码 | Team Orange | `submission/source/` | 当前控制策略和机构设计 |
| 简略提交包 | Team Orange | `submission/*.zip` | 只包含选手编写文件的提交包 |
| 完整本地运行包 | Team Orange 整合包 | `runtime/` | 主办方环境与当前选手文件的整合版本 |

### 目录结构

```text
.
├── README.md
├── rulebook/
│   └── Contest_Rules_final2026.pdf
├── official/
│   ├── ORGANIZER_FILES_INFO.txt
│   ├── IDC2026_CyberGame_Organizer_Complete_0814.zip
│   └── IDC2026_CyberGame_Organizer_Minimal_0814.zip
├── submission/
│   ├── source/
│   │   ├── user_side_go2.py
│   │   └── user_mechanism.xml
│   ├── CYBER_GAME_MINIMAL_PACKAGE_INFO.txt
│   └── IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip
└── runtime/
    ├── CYBER_GAME_COMPLETE_PACKAGE_INFO.txt
    └── IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip
```

### 主办方资料

#### 比赛规则书

`rulebook/Contest_Rules_final2026.pdf` 是 IDC2026 比赛规则书，版本为 1.01，日期为
2026 年 8 月 5 日。它是比赛规则的参考依据；本地运行包中的实现细节不能覆盖规则书。

#### 主办方完整环境

`official/IDC2026_CyberGame_Organizer_Complete_0814.zip` 是 0814 版本的主办方 Cyber
Game 环境基线。它包含主办方场地程序、`field_side.py`、`run_match.py`、MuJoCo 场景、
Go2 模型、人形模型、资源文件以及示例选手文件。

场地程序负责物理推进、比赛计时、随机目标、计分和裁判逻辑。这些文件放在仓库中，
是为了让访问者了解选手策略实际运行的比赛环境。

#### 主办方简略接口

`official/IDC2026_CyberGame_Organizer_Minimal_0814.zip` 是从同一 0814 环境中整理出的
选手接口文件副本，内容保持为主办方示例版本，包括：

- 主办方示例 `user_side_go2.py`；
- 主办方示例 `user_mechanism.xml`；
- 主办方的环境说明文件。

该压缩包用于对照接口，不是 Team Orange 的最终提交，也不包含完整场地运行环境。

### Team Orange 选手文件

当前选手编写的文件可以直接在 `submission/source/` 中查看：

- `user_side_go2.py`：Go2 控制策略、观测处理、执行器命令、红蓝方选择、行走行为和
  球处理逻辑；
- `user_mechanism.xml`：挂载到 Go2 模型上的选手机构设计。

这两个文件才代表 Team Orange 自行编写的策略和机构。它们不能单独启动比赛；主办方
场地程序会导入控制器、传入观测、应用执行器命令并推进 MuJoCo 物理仿真。

### 当前简略提交包

如果主办方只要求选手文件，使用
`submission/IDC2026_Team_Orange_Cyber_Game_Submission_Minimal.zip`。该包包含当前的
`user_side_go2.py`、`user_mechanism.xml` 和说明文件，不包含主办方场地程序、场景、模型
或资源文件。

### 当前完整运行包

如果需要在本地复现，使用
`runtime/IDC2026_Team_Orange_Cyber_Game_Submission_Complete.zip`。该包将 0814 环境
与当前 Team Orange 选手文件整合，并附带 PowerShell 启动脚本。它是 Team Orange 为了
复现而制作的整合包，不是主办方原始环境；正式评测时应优先使用主办方提供的更新版本。

评测环境要求为：

- Windows 11；
- Python 3.13.9；
- MuJoCo Python 包，包括图形运行所需的 `mujoco.viewer`；
- NumPy。

不需要 ONNX Runtime。压缩包不包含 Python 解释器和第三方 Python 包，这些依赖需要由
评测电脑预先安装或由主办方环境提供。

### 完整包运行方式

完整包按 PowerShell 准备。解压后，在包含 `run_match.ps1` 的 `IDC2026_CyberGame` 目录
中打开 PowerShell，运行：

```powershell
.\run_match.ps1 -Side red
.\run_match.ps1 -Side blue
```

手动运行时使用 PowerShell 的环境变量写法：

```powershell
$env:IDC2026_SIDE = "blue"
python .\run_match.py
```

不要在 PowerShell 中使用 CMD 的 `set IDC2026_SIDE=blue` 写法。

### 验证边界

当前选手文件已通过接口和 XML 测试，结果为 `12/12`。红蓝双方预检也已通过，包括机构
执行器接口、初始状态、Start B 包络和模型加载。压缩包完整性及解压后的加载也已检查。
这些是开发环境中的本地检查，不能替代主办方最终评测。

因此，访问者可以按下面的方式理解仓库：

- 到 `rulebook/` 查看比赛规则；
- 到 `official/` 了解主办方下发的赛题环境和运行基线；
- 到 `submission/source/` 查看 Team Orange 的策略和机构代码；
- 使用 `submission/*.zip` 进行只含选手文件的提交；
- 使用 `runtime/*.zip` 进行包含环境和选手代码的本地复现。
