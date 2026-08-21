"""IDC2026 Cyber Game 用户侧 Go2 控制器。

这份文件是参赛方的软件交付物，不是一个可以独立启动比赛的程序。官方
``run_match.py``/``field_side.py`` 会创建 MuJoCo 的 ``MjModel`` 和 ``MjData``，
再把场地侧允许公开的 observation 传给这里的 ``Go2UserController``；本文件
每个控制周期返回一个 ``{执行器名称: 目标值}`` 字典，最后仍由官方场地代码
写入 ``data.ctrl`` 并调用 ``mujoco.mj_step()``。

阅读这份代码时，可以按下面的顺序理解：

1. 文件开头的常量：红蓝侧、球位点、腿长、行走时间和发射标定参数；
2. ``_leg_*_kinematics``：把足端目标位置和三关节角度互相转换；
3. ``Go2UserController.user_control``：每帧的总调度入口；
4. ``_crawl_leg_command``：行走阶段生成 12 个 Go2 关节的目标角；
5. ``_mechanism_command``：机构状态机，负责识别目标、瞄准、开门、推球和收回；
6. 文件末尾的 binding/安全函数：把 XML 中的执行器名称绑定到 MuJoCo 数组，并在
   接口不匹配或 observation 异常时进入安全保持。

机器人携带两颗球，先用保守的四拍爬行到固定投放区域，再用两个实体推杆依次
发射。比赛运行时的决策只使用官方公开 observation；控制器不读取机器人基座
姿态、球的世界坐标、接触、得分真值或其他私有模型状态，也不负责推进物理。
"""

import os

import mujoco
import numpy as np


# ---------------------------------------------------------------------------
# 运行侧别和发射标定
# ---------------------------------------------------------------------------
# 官方运行器通过环境变量选择红方或蓝方。文件被导入时就检查侧别，避免后面
# 产生一整套看似正常、实际却朝向错误的控制命令。
SIDE = os.environ.get("IDC2026_SIDE", "red").strip().lower()
if SIDE not in {"red", "blue"}:
    raise ValueError(
        "IDC2026_SIDE must be exactly 'red' or 'blue'; "
        f"received {SIDE!r}."
    )


# 这些是冻结在用户文件中的常量，不依赖运行时读取实验 JSON。
# yaw/pitch offset 是额外标定量；球速只用于“估算俯仰角”，真正发射仍由 MuJoCo
# 中的推杆、挡门、接触和重力产生，而不是直接给球写速度。
TUNING_YAW_OFFSET = 0.0
TUNING_PITCH_OFFSET = 0.0
TUNING_BALL_SPEED = 6.0
TUNING_PUSH_STROKE = 0.200
# 推杆在有限行程内完成加速。时间越短，目标位置变化越快，通常会提高球速，
# 但也会让接触和执行器误差更敏感。
TUNING_PUSH_TIME = 0.36

# 目标槽位的俯仰微调表。键为“侧别、球色、目标槽位”，这样红蓝两侧和两个
# 目标可以分别校准，而不用把所有情况压缩成一个全局角度。
TUNING_PITCH_DEFAULTS = {
    ("red", "yellow", "red_1"): 0.02,
    ("red", "white", "red_1"): 0.04,
    ("red", "yellow", "red_2"): 0.025,
    ("red", "white", "red_2"): -0.03,
    ("blue", "yellow", "blue_1"): 0.03,
    ("blue", "yellow", "blue_2"): 0.0,
    ("blue", "white", "blue_1"): 0.0,
    ("blue", "white", "blue_2"): -0.04,
}
TUNING_YAW_DEFAULTS = {
    # 蓝方外侧黄色通道的一个小偏航修正。
    ("blue", "yellow", "blue_2"): -0.002,
}


def _goal_tuning_float(kind, color, goal_key):
    """读取当前侧别/颜色/槽位对应的最后一层标定修正。"""
    key = (SIDE, color, goal_key)
    return (
        TUNING_PITCH_DEFAULTS.get(key, 0.0)
        if kind == "PITCH"
        else TUNING_YAW_DEFAULTS.get(key, 0.0)
    )

MATCH_BALL_SUFFIX = "1" if SIDE == "red" else "2"
# 红方拥有 ``yellow_1/white_1``，蓝方拥有 ``yellow_2/white_2``。
# 蓝方朝向相反，因此把颜色映射到物理通道时进行镜像，避免两个球都靠近
# 目标框同一侧。这里的字符串必须和 XML 经 go2.xml attach 后的 user_* 名称对应。
PHYSICAL_LANE = (
    {"yellow": "yellow", "white": "white"}
    if SIDE == "red"
    else {"yellow": "white", "white": "yellow"}
)
# Go2 在两侧 Start B 的初始自由体状态。四元数顺序是 MuJoCo 使用的
# (w, x, y, z)，不是常见的 (x, y, z, w)。
START_POSITION = (
    np.array([-2.95, 3.34, 0.645], dtype=float)
    if SIDE == "red"
    else np.array([2.95, 3.34, 0.645], dtype=float)
)
START_YAW = np.pi if SIDE == "red" else 0.0
START_QUATERNION = (
    np.array([np.cos(START_YAW / 2.0), 0.0, 0.0, np.sin(START_YAW / 2.0)], dtype=float)
)

GO2_INITIAL_STATE = {
    "position": START_POSITION,
    "quaternion": START_QUATERNION,
    "linear_velocity": np.zeros(3, dtype=float),
    "angular_velocity": np.zeros(3, dtype=float),
}


def _ball_configuration():
    """返回四颗球的 reset 配置。

    当前参赛侧的两颗球通过 ``site`` 放入用户机构的槽位；对方的两颗球
    只是停放在场地上的固定初始位置。真正的世界坐标由官方 field side 在
    reset 时根据 site 和 Go2 当前姿态解析。
    """
    common = {
        "quaternion": np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        "linear_velocity": np.zeros(3, dtype=float),
        "angular_velocity": np.zeros(3, dtype=float),
    }
    if SIDE == "red":
        result = {
            "yellow_1": {"site": f"user_{PHYSICAL_LANE['yellow']}_slot"},
            "white_1": {"site": f"user_{PHYSICAL_LANE['white']}_slot"},
            "yellow_2": {"position": np.array([2.84, 3.29, 0.235])},
            "white_2": {"position": np.array([3.04, 3.29, 0.235])},
        }
    else:
        result = {
            "yellow_2": {"site": f"user_{PHYSICAL_LANE['yellow']}_slot"},
            "white_2": {"site": f"user_{PHYSICAL_LANE['white']}_slot"},
            "yellow_1": {"position": np.array([-2.84, 3.29, 0.235])},
            "white_1": {"position": np.array([-3.04, 3.29, 0.235])},
        }
    for config in result.values():
        config.update({key: value.copy() for key, value in common.items()})
        if "site" in config:
            config["position_offset"] = np.zeros(3, dtype=float)
    return result


BALL_INITIAL_CONDITIONS = _ball_configuration()

# Go2 本体的 12 个位置执行器，以及 XML 中要求出现的 6 个用户机构执行器。
# 官方 attach 会给机构原始名称加上 user_ 前缀，例如 turret_yaw ->
# user_turret_yaw。
LEG_ACTUATOR_PREFIXES = ("FR_", "FL_", "RR_", "RL_")
MECHANISM_ACTUATORS = (
    "user_turret_yaw",
    "user_turret_pitch",
    "user_yellow_gate",
    "user_white_gate",
    "user_yellow_pusher",
    "user_white_pusher",
)

LEG_NAMES = ("FL", "FR", "RL", "RR")
# 逆时针/镜像关系下使用的腿顺序。一次只抬一条腿，其他三条腿承担支撑。
CRAWL_SEQUENCE = ("RL", "FL", "RR", "FR")
# 单腿足端在髋关节坐标系中的默认目标，单位为米。
HOME_LEG_TARGET = np.array((0.0, 0.75, -1.50), dtype=float)
# 四条腿髋关节和大腿/小腿长度，来自 Go2 XML 的几何标定。
HIP_X = {"FL": 0.1934, "FR": 0.1934, "RL": -0.1934, "RR": -0.1934}
HIP_Y = {"FL": 0.0465, "FR": -0.0465, "RL": 0.0465, "RR": -0.0465}
HIP_LATERAL = {"FL": 0.0955, "FR": -0.0955, "RL": 0.0955, "RR": -0.0955}
THIGH_LENGTH = 0.213
CALF_LENGTH = 0.213

STABILIZE = "STABILIZE"
WALK_TO_TABLE = "WALK_TO_TABLE"
WALK_DOWN_RAMP = "WALK_DOWN_RAMP"
PLATFORM_SETTLE = "PLATFORM_SETTLE"
PLAN = "PLAN"
AIM_YELLOW = "AIM_YELLOW"
FIRE_YELLOW = "FIRE_YELLOW"
RECOVER = "RECOVER"
AIM_WHITE = "AIM_WHITE"
FIRE_WHITE = "FIRE_WHITE"
SAFE_HOLD = "SAFE_HOLD"

# 控制器的主状态机：先稳定，再行走，到投放区后读取目标并规划弹道，最后
# 依次发射两球。行走路线主要由时间驱动，而不是通过私有世界位姿闭环导航。
# WALK_DOWN_RAMP_TIME 当前为 0，表示到达标定点后直接进入平台稳定阶段。
WALK_TO_TABLE_TIME = 13.0
WALK_DOWN_RAMP_TIME = 0.0
PLATFORM_SETTLE_TIME = 0.30
WALK_CYCLE = 1.30
WALK_STEP_HEIGHT = 0.075

BASE_YAW = START_YAW
# 下面几组 bias 是机构通道、球心和目标框几何造成的经验修正。它们不是读取
# 世界状态；目标的基本方向仍由公开目标位置计算，bias 只修正机构误差。
YAW_BIAS = 0.20
COLOR_YAW_BIAS = {"yellow": 0.14, "white": 0.235}
PITCH_BIAS = 0.08
COLOR_PITCH_BIAS = {"yellow": 0.08, "white": 0.04}
# 两个目标槽位相距约 0.30 m。不同通道到不同槽位的误差可重复，因此显式记录
# “侧别 × 球色 × 槽位”的修正，不假设一个角度适用于所有目标。
GOAL_YAW_BIAS = {
    ("red", "yellow", "red_1"): 0.115,
    ("red", "yellow", "red_2"): 0.112,
    ("red", "white", "red_1"): 0.180,
    ("red", "white", "red_2"): 0.195,
    ("blue", "yellow", "blue_1"): -0.12,
    ("blue", "yellow", "blue_2"): -0.10,
    ("blue", "white", "blue_1"): -0.105,
    ("blue", "white", "blue_2"): -0.13,
}
GOAL_PITCH_BIAS = {
    ("red", "yellow", "red_1"): 0.08,
    ("red", "yellow", "red_2"): 0.05,
    ("red", "white", "red_1"): 0.04,
    ("red", "white", "red_2"): 0.04,
    # 蓝方是红方的镜像：偏航方向大体反号，槽位编号也跨中心线交换。
    ("blue", "yellow", "blue_1"): -0.14,
    ("blue", "yellow", "blue_2"): -0.123,
    ("blue", "white", "blue_1"): -0.53,
    ("blue", "white", "blue_2"): -0.40,
}
BASE_TARGET = np.array(
    (-1.70, 3.42, 0.0) if SIDE == "red" else (1.70, 3.24, 0.0),
    dtype=float,
)
# 发射机构的俯仰轴位于 base_link 上方；LAUNCH_Z 是稳定投放点处的标定球心高度。
# 它只用于抛体计算，不代表把球固定在世界坐标中。
LAUNCH_Z = 0.785
TURRET_PIVOT_X = -0.080
SLOT_X = 0.060
LANE_OFFSET = {"yellow": 0.070, "white": -0.070}
PUSH_STROKE = TUNING_PUSH_STROKE
GATE_OPEN = 0.250
PUSH_TIME = TUNING_PUSH_TIME
PRELOAD_TIME = 0.25
GATE_RELEASE_DELAY = 0.12
BLUE_AIM_MIN_TIME = 1.60
RECOVER_TIME = 0.46
BALL_SPEED_FOR_AIM = TUNING_BALL_SPEED
GRAVITY = 9.81


def _smoothstep(value):
    """平滑插值函数。

    线性插值在开始/结束时速度突变，容易给位置执行器和接触求解器带来冲击。
    smoothstep 在 0 和 1 附近斜率为 0，用于腿抬放、推杆和挡门的缓启动/停止。
    """
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def _normalize_angle(angle):
    """把角度折返到 [-pi, pi)。"""
    return float((float(angle) + np.pi) % (2.0 * np.pi) - np.pi)


def _leg_forward_kinematics(leg, joint_angles):
    """正运动学：三关节角 -> 足端在髋关节坐标系中的 (x, y, z)。

    这里的坐标不是世界坐标。``_crawl_leg_command`` 先在髋关节局部坐标系
    里规划足端轨迹，再用逆运动学得到三个目标关节角。
    """
    q1, q2, q3 = (float(v) for v in joint_angles)
    lateral = HIP_LATERAL[leg]
    x = -THIGH_LENGTH * np.sin(q2) - CALF_LENGTH * np.sin(q2 + q3)
    sagittal_z = -THIGH_LENGTH * np.cos(q2) - CALF_LENGTH * np.cos(q2 + q3)
    y = np.cos(q1) * lateral - np.sin(q1) * sagittal_z
    z = np.sin(q1) * lateral + np.cos(q1) * sagittal_z
    return np.array((x, y, z), dtype=float)


def _leg_inverse_kinematics(leg, foot_in_hip):
    """逆运动学：足端局部目标 -> Go2 的髋/大腿/小腿角度。

    计算分为两步：先在髋关节外展方向求髋角，再把剩余的二维问题当作由
    大腿和小腿组成的两连杆求解。不可达或超出 XML 关节范围时抛异常，调用
    方会退回 HOME_LEG_TARGET，避免向 MuJoCo 写入非法目标。
    """
    x, y, z = (float(v) for v in foot_in_hip)
    lateral = HIP_LATERAL[leg]
    radial_sq = y * y + z * z
    if radial_sq <= lateral * lateral:
        raise ValueError("unreachable lateral target")
    sagittal_z = -np.sqrt(max(radial_sq - lateral * lateral, 1.0e-12))
    q1 = _normalize_angle(np.arctan2(z, y) - np.arctan2(sagittal_z, lateral))
    reach_sq = x * x + sagittal_z * sagittal_z
    cosine_knee = (
        reach_sq - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH
    ) / (2.0 * THIGH_LENGTH * CALF_LENGTH)
    if cosine_knee < -1.00001 or cosine_knee > 1.00001:
        raise ValueError("unreachable sagittal target")
    q3 = -np.arccos(float(np.clip(cosine_knee, -1.0, 1.0)))
    alpha = np.arctan2(-x, -sagittal_z)
    beta = np.arctan2(
        CALF_LENGTH * np.sin(q3), THIGH_LENGTH + CALF_LENGTH * np.cos(q3)
    )
    result = np.array((q1, alpha - beta, q3), dtype=float)
    limits = {
        "hip": (-1.0472, 1.0472),
        "thigh": (-1.5708, 3.4907) if leg[0] == "F" else (-0.5236, 4.5379),
        "calf": (-2.7227, -0.83776),
    }
    for value, (low, high) in zip(result, limits.values()):
        if value < low - 1.0e-5 or value > high + 1.0e-5:
            raise ValueError("joint target outside Go2 limits")
    return result


class Go2UserController:
    """用户侧总控制器。

    官方场地把 ``model``/``data`` 传进来；本类只建立执行器索引、读取公开
    状态、计算控制目标和返回命令。它不创建第二个 ``MjData``，也不调用
    ``mj_step``，因为物理推进权属于官方 field side。

    核心状态链为：

    ``STABILIZE -> WALK_TO_TABLE -> WALK_DOWN_RAMP -> PLATFORM_SETTLE``
    ``-> PLAN -> AIM_YELLOW -> FIRE_YELLOW -> RECOVER``
    ``-> AIM_WHITE -> FIRE_WHITE -> SAFE_HOLD``。
    """

    def __init__(self, model, data):
        """绑定 MuJoCo 模型并准备控制器内部状态。

        这里不假设执行器在数组中的固定编号，而是通过名字扫描。这样可以
        适应官方编译后的 actuator 顺序，同时能检查 XML 是否确实提供了
        12 个 Go2 腿执行器和 6 个用户机构执行器。
        """
        self.model = model
        self.data = data
        self.go2_joints = self._discover_go2_joints()
        self.user_mechanism_joints = self._discover_user_joints()
        self._validate_mechanism_interface()
        self._configure_go2_position_servos()
        self.go2_joint_index = {
            item["joint_name"]: index for index, item in enumerate(self.go2_joints)
        }
        self.position_kp = np.array(
            [float(self.model.actuator_gainprm[item["actuator_id"], 0]) for item in self.go2_joints],
            dtype=float,
        )
        self.integral_gain = np.array(
            [0.5 if item["joint_name"].endswith("_hip_joint") else 3.0 for item in self.go2_joints],
            dtype=float,
        )
        self.integral_torque_limit = np.array(
            [0.5 if item["joint_name"].endswith("_hip_joint") else 2.0 for item in self.go2_joints],
            dtype=float,
        )
        self.integral_error = np.zeros(12, dtype=float)
        self.q_stand = np.array(
            [self._stand_angle(item["actuator_name"]) for item in self.go2_joints],
            dtype=float,
        )
        self.verbose = os.environ.get("IDC2026_VERBOSE", "0") == "1"
        self.reset()

    # ------------------------------------------------------------------
    # 官方接口：field side -> observation -> user_control -> command
    # ------------------------------------------------------------------
    def get_observation(self, field_observation):
        """把官方 field observation 包装成用户侧约定的数据结构。

        Go2 和机构关节的 qpos/qvel 来自当前共享的 ``data``；场地信息则由
        官方代码传入。这里不把 base pose、球位姿、接触或得分真值塞进用户
        observation，保持参赛接口边界。
        """
        go2_q, go2_dq = {}, {}
        for item in self.go2_joints:
            name = item["actuator_name"]
            go2_q[name] = float(self.data.qpos[item["qpos_adr"]])
            go2_dq[name] = float(self.data.qvel[item["qvel_adr"]])
        user_q, user_dq = {}, {}
        for item in self.user_mechanism_joints:
            name = item["actuator_name"]
            user_q[name] = float(self.data.qpos[item["qpos_adr"]])
            user_dq[name] = float(self.data.qvel[item["qvel_adr"]])
        return {
            "go2": {"joint_position": go2_q, "joint_velocity": go2_dq},
            "user_mechanism": {"joint_position": user_q, "joint_velocity": user_dq},
            "field": field_observation,
        }

    def compute_control(self, sim_time, dt, field_observation):
        """官方调用的适配层。

        ``compute_control`` 负责拼 observation、调用用户策略，并检查返回值
        是字典。真正的策略入口是 ``user_control``。
        """
        command = self.user_control(self.get_observation(field_observation), sim_time, dt)
        if not isinstance(command, dict):
            raise TypeError("user_control() must return an actuator command dict.")
        return command

    def user_control(self, observation, sim_time, dt):
        """每个 MuJoCo 控制周期生成完整 actuator 命令。

        行走状态返回腿部步态命令；其余状态让 Go2 保持站立。随后统一叠加
        机构命令，因此最终返回值同时包含 12 个 Go2 关节和 6 个机构执行器。
        """
        sim_time = float(sim_time)
        if self.state in {WALK_TO_TABLE, WALK_DOWN_RAMP}:
            command = self._crawl_leg_command(sim_time, observation, dt)
        else:
            command = self._standing_leg_command()
        command.update(self._mechanism_command(observation, sim_time))
        return command

    # Leg control -----------------------------------------------------------
    def _standing_leg_command(self):
        """让 12 个 Go2 位置执行器回到站立角度。"""
        return {
            item["actuator_name"]: float(self.q_stand[index])
            for index, item in enumerate(self.go2_joints)
        }

    def _crawl_leg_command(self, sim_time, observation, dt):
        """用四拍、一次一腿的方式生成 Go2 行走目标。

        这是一个以时间为主的开环步态：每条腿都有相位偏移，摆动腿先抬高、
        向前摆、放下，支撑腿则在身体下方缓慢回摆。髋部横向偏移用于在抬腿
        前把重心移向其余三条支撑腿。

        观测中的当前关节角只用于积分补偿和误差计算，不用于读取世界位置来
        重新规划路线。路线时间由 ``walk_start_time`` 和常量控制。
        """
        # 当前 12 个关节角，用于计算位置误差和积累小幅积分补偿。
        q_now = np.array(
            [observation["go2"]["joint_position"][item["actuator_name"]]
             for item in self.go2_joints], dtype=float
        )
        # elapsed 决定已经走了多久；phase 是一个 0~1 的周期相位；blend 让
        # 行走从站立姿态平滑起步，避免第一步突然跳到大腿目标角。
        elapsed = max(0.0, sim_time - self.walk_start_time)
        phase = (elapsed / WALK_CYCLE) % 1.0
        blend = _smoothstep(elapsed / 0.60)
        nominal_x = _leg_forward_kinematics("FL", HOME_LEG_TARGET)[0]
        nominal_z = _leg_forward_kinematics("FL", HOME_LEG_TARGET)[2]
        # q_des 是本周期要发给位置执行器的 12 维目标角。摆动腿不做积分，
        # 避免摆动过程中把短时误差积累成落地后的错误偏置。
        q_des = self.q_stand.copy()
        integrate_mask = np.ones(12, dtype=bool)
        phase_offsets = {"FR": 0.00, "RL": 0.25, "FL": 0.50, "RR": 0.75}
        step_length = -0.130
        half_step = 0.5 * step_length
        common_hip_shift = 0.0

        for leg in ("FR", "FL", "RR", "RL"):
            local = (phase - phase_offsets[leg]) % 1.0
            if local < 0.25:
                # 该腿处于摆动段：前 20% 抬腿，中间 60% 向前摆，最后 20%
                # 放腿。每个子段仍用 smoothstep 过渡。
                u = local / 0.25
                if u < 0.20:
                    lift_blend = _smoothstep(u / 0.20)
                    hip_shift_mag = lift_blend
                elif u < 0.80:
                    lift_blend = 1.0
                    hip_shift_mag = 1.0
                else:
                    lift_blend = 1.0 - _smoothstep((u - 0.80) / 0.20)
                    hip_shift_mag = lift_blend
                if u < 0.20:
                    x_foot, z_foot = nominal_x - half_step, nominal_z
                elif u < 0.35:
                    s = _smoothstep((u - 0.20) / 0.15)
                    x_foot, z_foot = nominal_x - half_step, nominal_z + WALK_STEP_HEIGHT * s
                elif u < 0.65:
                    s = _smoothstep((u - 0.35) / 0.30)
                    x_foot = nominal_x - half_step + step_length * s
                    z_foot = nominal_z + WALK_STEP_HEIGHT
                elif u < 0.80:
                    s = _smoothstep((u - 0.65) / 0.15)
                    x_foot, z_foot = nominal_x + half_step, nominal_z + WALK_STEP_HEIGHT * (1.0 - s)
                else:
                    x_foot, z_foot = nominal_x + half_step, nominal_z
                side_sign = -1.0 if leg in ("FR", "RR") else 1.0
                hip_shift = 0.055 * side_sign * hip_shift_mag
                airborne = u > 0.20 and u < 0.80
            else:
                # 支撑段：足端接触地面，身体前进后让该腿向后回摆，准备
                # 下一次成为摆动腿。
                stance = (local - 0.25) / 0.75
                x_foot = nominal_x + half_step - step_length * _smoothstep(stance)
                z_foot = nominal_z
                hip_shift, airborne = 0.0, False
            x_cmd = nominal_x + blend * (x_foot - nominal_x)
            z_cmd = nominal_z + blend * (z_foot - nominal_z)
            # 足端轨迹必须先经过逆运动学；如果由于数值误差/几何不可达，
            # 则放弃本次特殊目标并回到安全站立腿型。
            try:
                planar = _leg_inverse_kinematics(
                    leg, (x_cmd, HIP_LATERAL[leg] + hip_shift, z_cmd)
                )
            except ValueError:
                planar = HOME_LEG_TARGET.copy()
            for part, index, value in zip(("hip", "thigh", "calf"),
                                          (self.go2_joint_index[f"{leg}_hip_joint"],
                                           self.go2_joint_index[f"{leg}_thigh_joint"],
                                           self.go2_joint_index[f"{leg}_calf_joint"]),
                                          planar):
                q_des[index] = value
            if airborne:
                integrate_mask[[self.go2_joint_index[f"{leg}_{part}_joint"] for part in ("hip", "thigh", "calf")]] = False
            common_hip_shift = hip_shift if abs(hip_shift) > abs(common_hip_shift) else common_hip_shift

        # 所有腿共享同一方向的最大髋偏移，用来近似保持身体横向重心稳定。
        for leg in LEG_NAMES:
            q_des[self.go2_joint_index[f"{leg}_hip_joint"]] = common_hip_shift
        # 位置伺服已经负责主要跟踪，这里只用一个限幅积分项补偿负载导致的
        # 稳态误差；误差过大时不积分，避免机器人摔倒后继续“记住”旧误差。
        error = q_des - q_now
        valid = integrate_mask & (np.abs(error) <= 0.25)
        self.integral_error[~integrate_mask] = 0.0
        self.integral_error[valid] += error[valid] * dt
        tau_i = np.clip(self.integral_gain * self.integral_error,
                        -self.integral_torque_limit, self.integral_torque_limit)
        nz = self.integral_gain > 0.0
        self.integral_error[nz] = tau_i[nz] / self.integral_gain[nz]
        q_cmd = q_des + tau_i / self.position_kp
        return {item["actuator_name"]: float(q_cmd[index]) for index, item in enumerate(self.go2_joints)}

    # ------------------------------------------------------------------
    # 机构状态机：目标规划、瞄准、开门、推球和收回
    # ------------------------------------------------------------------
    def _mechanism_command(self, observation, sim_time):
        """根据 observation 生成 6 个机构执行器的命令。

        每次调用都先把所有机构命令置零，再只覆盖当前需要的挡门/推杆/云台。
        这样即使状态切换或 observation 异常，也不会遗留上一个阶段的推杆
        目标。流程是：

        * STABILIZE：开局站稳；
        * WALK_TO_TABLE / WALK_DOWN_RAMP：让腿部控制器完成路线；
        * PLAN：读取本方两个目标的公开颜色和位置，计算两条抛体方案；
        * AIM_*：把云台移动到对应角度，并等待机构稳定；
        * FIRE_*：预压推杆、抬挡门、推进球；
        * RECOVER：黄球发射后收回推杆，再准备白球；
        * SAFE_HOLD：关闭/收回机构并保持安全状态。
        """
        q = observation.get("user_mechanism", {}).get("joint_position", {})
        dq = observation.get("user_mechanism", {}).get("joint_velocity", {})
        field = observation.get("field", {})
        # 默认值是安全/待机位置。后续分支只修改当前球对应的通道。
        command = {
            "user_turret_yaw": 0.0,
            "user_turret_pitch": 0.0,
            "user_yellow_gate": 0.0,
            "user_white_gate": 0.0,
            "user_yellow_pusher": 0.0,
            "user_white_pusher": 0.0,
        }
        # 机构关节状态缺失时，不能可靠判断挡门和推杆是否已经到位，
        # 因此直接进入 SAFE_HOLD，而不是盲目发射。
        if not self._valid_joint_observation(q, dq):
            self._fail_safe("mechanism joint observation missing or non-finite", sim_time)
            return command
        # 剩余比赛时间是官方公开的场地信息。最后 5 秒不再启动新的机构动作，
        # 给正在运行的动作留出安全收尾时间。
        remaining = field.get("remaining_time_s")
        if not isinstance(remaining, (int, float, np.integer, np.floating)):
            self._fail_safe("remaining match time is missing", sim_time)
            return command
        if self.state != SAFE_HOLD and float(remaining) < 5.0:
            self._fail_safe("insufficient match time", sim_time)
            return command

        # 这些是只依赖仿真时间的阶段切换。进入 PLAN 后才读取随机目标，
        # 避免开局姿态尚未稳定就开始规划发射。
        if self.state == STABILIZE and sim_time >= 1.0:
            self._transition(WALK_TO_TABLE, sim_time)
            self.walk_start_time = sim_time
        elif self.state == WALK_TO_TABLE and self._state_elapsed(sim_time) >= WALK_TO_TABLE_TIME:
            self._transition(WALK_DOWN_RAMP, sim_time)
            self.walk_start_time = sim_time
        elif self.state == WALK_DOWN_RAMP and self._state_elapsed(sim_time) >= WALK_DOWN_RAMP_TIME:
            self._transition(PLATFORM_SETTLE, sim_time)
        elif self.state == PLATFORM_SETTLE and self._state_elapsed(sim_time) >= PLATFORM_SETTLE_TIME:
            self._transition(PLAN, sim_time)

        if self.state == PLAN:
            # _build_plan 会同时验证目标数量、颜色和位置，并为 yellow/white
            # 各生成一份独立的 yaw/pitch/goal_key 方案。
            if self._build_plan(field):
                self._transition(AIM_YELLOW, sim_time)
            else:
                self._fail_safe(self.plan_error or "invalid goal plan", sim_time)

        if self.state == AIM_YELLOW:
            command.update(self._aim_command("yellow", q))
            if self._aim_ready(q, dq, "yellow", sim_time):
                self.fire_phase = "OPEN"
                self.phase_start_time = sim_time
                self._transition(FIRE_YELLOW, sim_time)
        elif self.state == FIRE_YELLOW:
            command.update(self._fire_command(q, "yellow", sim_time))
        elif self.state == RECOVER:
            command.update(self._recover_command(q, "yellow", sim_time))
        elif self.state == AIM_WHITE:
            command.update(self._aim_command("white", q))
            if self._aim_ready(q, dq, "white", sim_time):
                self.fire_phase = "OPEN"
                self.phase_start_time = sim_time
                self._transition(FIRE_WHITE, sim_time)
        elif self.state == FIRE_WHITE:
            command.update(self._fire_command(q, "white", sim_time))
        elif self.state == SAFE_HOLD:
            command.update(self._safe_hold_command(q))
        return command

    def _build_plan(self, field):
        """从官方公开目标信息建立两球发射计划。

        官方每次 reset 可能交换目标颜色，因此这里不能写死“左边黄色、右边
        白色”。函数只接受本方的 ``SIDE_1``/``SIDE_2`` 两个目标，并要求恰好
        一个 yellow 和一个 white；每个目标的位置再交给抛体估算器。
        """
        self.plan_error = None
        goals = field.get("goals")
        if not isinstance(goals, dict):
            self.plan_error = "goal observation is missing"
            return False
        selected = []
        for key, entry in goals.items():
            # 忽略对方目标和未知字段，避免控制器误把对方槽位当成自己的目标。
            if not isinstance(entry, dict) or entry.get("team") != SIDE:
                continue
            if key not in {f"{SIDE}_1", f"{SIDE}_2"}:
                continue
            color = entry.get("color")
            position = np.asarray(entry.get("position", []), dtype=float)
            if color not in {"yellow", "white"} or position.shape != (3,):
                self.plan_error = f"invalid goal entry {key!r}"
                return False
            if not np.all(np.isfinite(position)):
                self.plan_error = f"non-finite goal entry {key!r}"
                return False
            selected.append((key, color, position.copy()))
        if len(selected) != 2 or sorted(item[1] for item in selected) != ["white", "yellow"]:
            self.plan_error = "team must expose exactly one yellow and one white goal"
            return False
        self.plan = {}
        for key, color, target in selected:
            # plan 按球色索引，后面发射时可以固定 yellow -> white，而目标槽位
            # 仍保存在 solution["goal_key"] 中用于选择标定参数。
            solution = self._ballistic_solution(key, color, target)
            if solution is None:
                self.plan_error = f"no bounded ballistic solution for {color}"
                return False
            solution["goal_key"] = key
            self.plan[color] = solution
        return True

    def _ballistic_solution(self, goal_key, color, target):
        """根据目标位置估算云台 yaw/pitch。

        这里使用的是简化的固定球速抛体模型：

        * yaw：从发射点水平投影指向目标，再叠加通道/槽位标定；
        * pitch：在一组候选仰角中搜索，使理论高度变化最接近目标；
        * 最后把角度限制在 XML 中的 turret 关节范围内。

        这只是“把机构摆到哪里”的开环标定，不会把球直接传送到目标；球仍由
        MuJoCo 中的挡门、推杆、碰撞和重力实际飞行。
        """
        # BASE_TARGET 是预先标定的 Go2 发射位置，slot_local 是球槽相对于
        # 机构俯仰/偏航转轴的局部坐标。两者合成发射点的水平位置。
        base_angle = BASE_YAW
        rotation = np.array(
            [[np.cos(base_angle), -np.sin(base_angle)],
             [np.sin(base_angle), np.cos(base_angle)]]
        )
        lane = PHYSICAL_LANE[color]
        slot_local = np.array((TURRET_PIVOT_X + SLOT_X, LANE_OFFSET[lane]))
        origin_xy = BASE_TARGET[:2] + rotation @ slot_local
        horizontal = np.asarray(target[:2], dtype=float) - origin_xy
        distance = float(np.linalg.norm(horizontal))
        # 距离太近/太远都不在当前标定和目标物理范围内，直接让上层进入安全态。
        if distance < 0.25 or distance > 4.5:
            return None
        yaw = _normalize_angle(
            np.arctan2(horizontal[1], horizontal[0]) - BASE_YAW
            + GOAL_YAW_BIAS.get(
                (SIDE, color, goal_key), COLOR_YAW_BIAS.get(color, YAW_BIAS)
            )
            + TUNING_YAW_OFFSET
            + _goal_tuning_float("YAW", color, goal_key)
        )
        if abs(yaw) > 2.95:
            return None

        # 在有限候选仰角中选择理论高度误差最小的一项。目标位置来自公开
        # observation；实际球路仍由后续 MuJoCo 物理决定。
        dz = float(target[2] - LAUNCH_Z)
        best = None
        for elevation in np.linspace(-0.25, 0.65, 181):
            c = np.cos(elevation)
            if c <= 0.1:
                continue
            predicted = distance * np.tan(elevation) - (
                GRAVITY * distance * distance / (2.0 * BALL_SPEED_FOR_AIM**2 * c * c)
            )
            error = abs(predicted - dz)
            if best is None or error < best[0]:
                best = (error, elevation)
        if best is None or best[0] > 0.16:
            return None
        return {
            "yaw": float(yaw),
            # 在组合后的 Go2 局部坐标中，云台铰链的正方向与世界仰角约定相反；
            # 对这个 XML 轴来说，正的 pitch 关节角会抬高炮口。符号不能只
            # 看数学公式，必须和 user_mechanism.xml 的 axis 一起理解。
            "pitch": float(
                np.clip(
                    best[1]
                    + GOAL_PITCH_BIAS.get(
                        (SIDE, color, goal_key), COLOR_PITCH_BIAS.get(color, PITCH_BIAS)
                    ),
                    -0.70,
                    0.95,
                )
                + TUNING_PITCH_OFFSET
                + _goal_tuning_float("PITCH", color, goal_key)
            ),
            "target": np.asarray(target, dtype=float).copy(),
        }

    def _aim_command(self, color, q):
        """瞄准阶段：固定云台角度并保持两个挡门关闭、推杆归零。"""
        target = self.plan[color]
        other = "white" if color == "yellow" else "yellow"
        return {
            "user_turret_yaw": target["yaw"],
            "user_turret_pitch": target["pitch"],
            "user_yellow_gate": 0.0,
            "user_white_gate": 0.0,
            "user_yellow_pusher": 0.0,
            "user_white_pusher": 0.0,
        }

    def _aim_ready(self, q, dq, color, sim_time):
        """判断云台和当前发射通道是否已经稳定到可以发射。

        红方要求姿态/速度连续满足阈值若干个控制周期；蓝方由于镜像机构的
        振铃更明显，采用最短等待时间加稍宽的姿态容差。
        """
        target = self.plan[color]
        lane = PHYSICAL_LANE[color]
        # 俯仰/偏航两级铰链还承载着两条装满球的通道，蓝方镜像姿态下可能在
        # 进入标定角度后仍有较长时间的小幅俯仰振铃。因此蓝方使用一个有限
        # 等待窗口和稍宽的姿态容差；随后推杆预压的 0.25 s 还会在挡门关闭
        # 时给云台额外的收敛时间。
        if SIDE == "blue" and self._state_elapsed(sim_time) >= BLUE_AIM_MIN_TIME:
            return (
                abs(q["user_turret_yaw"] - target["yaw"]) < 0.08
                and abs(q["user_turret_pitch"] - target["pitch"]) < 0.18
                and q[f"user_{lane}_pusher"] < 0.012
                and q[f"user_{lane}_gate"] < 0.004
            )
        return self._continuous(
            abs(q["user_turret_yaw"] - target["yaw"]) < 0.018
            and abs(q["user_turret_pitch"] - target["pitch"]) < 0.018
            and abs(dq["user_turret_yaw"]) < 0.10
            and abs(dq["user_turret_pitch"]) < 0.10
            and q[f"user_{lane}_pusher"] < 0.012
            and abs(dq[f"user_{lane}_pusher"]) < 0.10
            and q[f"user_{lane}_gate"] < 0.004
            and abs(dq[f"user_{lane}_gate"]) < 0.06,
            20,
        )

    def _fire_command(self, q, color, sim_time):
        """执行一个球的物理发射时序。

        ``OPEN -> UNLOCK -> PUSH -> DONE`` 的顺序很重要：

        1. OPEN：挡门仍关闭，推杆先预压到球后方；
        2. UNLOCK：抬起挡门，但推杆继续挡住球，避免挡门撞球；
        3. PUSH：挡门保持打开，推杆在有限行程内平滑前进；
        4. DONE：等待球离开，再转到收回或第二球。
        """
        target = self.plan[color]
        lane = PHYSICAL_LANE[color]
        pusher = f"user_{lane}_pusher"
        gate = f"user_{lane}_gate"
        elapsed = self._phase_elapsed(sim_time)
        command = {
            "user_turret_yaw": target["yaw"],
            "user_turret_pitch": target["pitch"],
            "user_yellow_gate": 0.0,
            "user_white_gate": 0.0,
            "user_yellow_pusher": 0.0,
            "user_white_pusher": 0.0,
        }
        if self.fire_phase == "OPEN":
            # 挡门仍关闭时，先让推杆轻轻顶住球。这样云台继续稳定时，球不会
            # 因通道倾斜而提前滚出。
            command[gate] = 0.0
            command[pusher] = 0.100 * _smoothstep(elapsed / PRELOAD_TIME)
            if elapsed >= PRELOAD_TIME:
                self.fire_phase = "UNLOCK"
                self.phase_start_time = sim_time
        elif self.fire_phase == "UNLOCK":
            # 推杆继续挡住球时抬起挡门，避免挡门上升过程中直接把球向上打飞。
            command[gate] = GATE_OPEN
            command[pusher] = 0.100
            if self._phase_elapsed(sim_time) >= GATE_RELEASE_DELAY:
                self.fire_phase = "PUSH"
                self.phase_start_time = sim_time
        elif self.fire_phase == "PUSH":
            command[gate] = GATE_OPEN
            fraction = _smoothstep(self._phase_elapsed(sim_time) / PUSH_TIME)
            command[pusher] = 0.100 + (PUSH_STROKE - 0.100) * fraction
            if self._phase_elapsed(sim_time) >= PUSH_TIME + 0.35:
                self.fire_phase = "DONE"
                self.phase_start_time = sim_time
        else:
            command[gate] = GATE_OPEN
            command[pusher] = PUSH_STROKE
            if self._phase_elapsed(sim_time) >= 0.25:
                if color == "yellow":
                    self._transition(RECOVER, sim_time)
                else:
                    self._transition(SAFE_HOLD, sim_time)
        return command

    def _recover_command(self, q, color, sim_time):
        """第一球发射后收回推杆，为第二球清出通道。"""
        lane = PHYSICAL_LANE[color]
        pusher = f"user_{lane}_pusher"
        gate = f"user_{lane}_gate"
        elapsed = self._state_elapsed(sim_time)
        # 某个蓝方布局中第二球的飞行余量更短，因此第一球推杆可更快回零；
        # 其余布局采用更保守的回收时间，给云台和机构更多稳定时间。
        recover_time = RECOVER_TIME
        if (
            SIDE == "blue"
            and color == "yellow"
            and self.plan.get("white", {}).get("goal_key") == "blue_1"
        ):
            recover_time = 0.60
        fraction = 1.0 - _smoothstep(elapsed / recover_time)
        command = {
            "user_turret_yaw": self.plan[color]["yaw"],
            "user_turret_pitch": self.plan[color]["pitch"],
            "user_yellow_gate": 0.0,
            "user_white_gate": 0.0,
            "user_yellow_pusher": 0.0,
            "user_white_pusher": 0.0,
        }
        command[pusher] = PUSH_STROKE * max(0.0, fraction)
        # 整个回收过程中挡门保持抬起。过早关闭会让挡门和推杆在同一 x 区域
        # 发生机械干涉，把推杆卡在半程，第二球就无法进入 AIM_WHITE。
        command[gate] = GATE_OPEN
        if elapsed >= recover_time and q[pusher] < 0.015:
            self._transition(AIM_WHITE, sim_time)
        return command

    def _safe_hold_command(self, q):
        """安全保持：云台回中、挡门关闭、两个推杆回零。"""
        return {
            "user_turret_yaw": 0.0,
            "user_turret_pitch": 0.0,
            "user_yellow_gate": 0.0,
            "user_white_gate": 0.0,
            "user_yellow_pusher": 0.0,
            "user_white_pusher": 0.0,
        }

    # ------------------------------------------------------------------
    # 官方 reset 钩子和机构初始状态
    # ------------------------------------------------------------------
    def initial_robot_state(self):
        """向官方返回 Go2 初始自由体状态，并重新归一化四元数。"""
        state = {
            key: np.asarray(value, dtype=float).copy()
            for key, value in GO2_INITIAL_STATE.items()
        }
        norm = float(np.linalg.norm(state["quaternion"]))
        if norm <= 1.0e-12:
            raise ValueError("Initial Go2 quaternion must not be zero.")
        state["quaternion"] /= norm
        return state

    def initial_ball_state(self):
        """向官方返回四颗球的初始配置副本，避免调用方修改全局常量。"""
        return {
            name: {
                key: value.copy() if isinstance(value, np.ndarray) else str(value)
                for key, value in config.items()
            }
            for name, config in BALL_INITIAL_CONDITIONS.items()
        }

    def initial_joint_state(self):
        """返回 Go2 关节的 qpos/qvel 地址、执行器编号和站立角。"""
        return [
            (item["qpos_adr"], item["qvel_adr"], item["actuator_id"], float(self.q_stand[index]))
            for index, item in enumerate(self.go2_joints)
        ]

    def reset(self):
        """清空一次比赛的状态机、计时器、规划结果和安全标志。"""
        self.state = STABILIZE
        self.state_start_time = 0.0
        self.walk_start_time = 0.0
        self.phase_start_time = 0.0
        self.fire_phase = None
        self.plan = {}
        self.plan_error = None
        self.failure_reason = None
        self._ready_steps = 0

    def handle_key(self, key):
        """用户侧没有额外键盘控制；官方接口要求保留该钩子。"""
        del key
        return False

    # ------------------------------------------------------------------
    # XML 执行器绑定、位置伺服配置和安全检查
    # ------------------------------------------------------------------
    def _discover_go2_joints(self):
        """按命名空间找到 12 个 Go2 腿 actuator。"""
        result = []
        for actuator_id in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if name is None or not name.startswith(LEG_ACTUATOR_PREFIXES):
                continue
            item = self._joint_actuator_record(actuator_id, name)
            if item is not None:
                result.append(item)
        if len(result) != 12:
            raise RuntimeError(f"Expected 12 Go2 leg actuators, found {len(result)}.")
        return result

    def _discover_user_joints(self):
        """找到 attach 后名称以 user_ 开头的机构关节 actuator。"""
        result = []
        for actuator_id in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
            if name is None or not name.startswith("user_"):
                continue
            item = self._joint_actuator_record(actuator_id, name)
            if item is not None:
                result.append(item)
        return result

    def _joint_actuator_record(self, actuator_id, actuator_name):
        """把 MuJoCo actuator 编号转换成控制器需要的索引记录。"""
        if self.model.actuator_trntype[actuator_id] != mujoco.mjtTrn.mjTRN_JOINT:
            return None
        joint_id = int(self.model.actuator_trnid[actuator_id, 0])
        return {
            "actuator_id": actuator_id,
            "actuator_name": actuator_name,
            "joint_id": joint_id,
            "joint_name": mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            "qpos_adr": int(self.model.jnt_qposadr[joint_id]),
            "qvel_adr": int(self.model.jnt_dofadr[joint_id]),
        }

    def _validate_mechanism_interface(self):
        """确保 XML 暴露的机构接口与控制器预期完全一致。"""
        found = {item["actuator_name"] for item in self.user_mechanism_joints}
        expected = set(MECHANISM_ACTUATORS)
        if found != expected:
            raise RuntimeError(
                "Mechanism actuator interface mismatch: "
                f"missing={sorted(expected - found)}, extra={sorted(found - expected)}."
            )

    def _configure_go2_position_servos(self):
        """为 Go2 位置执行器设置本版本使用的 kp/kd。"""
        for item in self.go2_joints:
            name = item["actuator_name"]
            kp, kd = (60.0, 4.0) if name.endswith("_hip") else (90.0, 6.0)
            actuator_id = item["actuator_id"]
            self.model.actuator_gainprm[actuator_id, 0] = kp
            self.model.actuator_biasprm[actuator_id, 1] = -kp
            self.model.actuator_biasprm[actuator_id, 2] = -kd

    @staticmethod
    def _stand_angle(name):
        """按执行器名称返回站立姿态的目标角。"""
        if name.endswith("_hip"):
            return 0.0
        if name.endswith("_thigh"):
            return 0.75
        if name.endswith("_calf"):
            return -1.50
        raise RuntimeError(f"Unknown Go2 leg actuator {name!r}.")

    @staticmethod
    def _valid_joint_observation(q, dq):
        """检查 6 个机构关节的位置/速度是否都存在且为有限数。"""
        return all(
            name in q and name in dq and np.isfinite(q[name]) and np.isfinite(dq[name])
            for name in MECHANISM_ACTUATORS
        )

    def _continuous(self, condition, required_steps):
        """把瞬时条件变成连续满足 N 个控制周期的稳定条件。"""
        self._ready_steps = self._ready_steps + 1 if condition else 0
        return self._ready_steps >= required_steps

    def _state_elapsed(self, sim_time):
        """返回当前状态已经持续的仿真时间。"""
        return max(0.0, float(sim_time) - self.state_start_time)

    def _phase_elapsed(self, sim_time):
        """返回当前发射子阶段已经持续的仿真时间。"""
        return max(0.0, float(sim_time) - self.phase_start_time)

    def _transition(self, new_state, sim_time):
        """切换主状态并重置状态内计时/稳定计数。"""
        if self.verbose:
            print(f"[USER] {self.state} -> {new_state} at {float(sim_time):.3f} s")
        self.state = new_state
        self.state_start_time = float(sim_time)
        self._ready_steps = 0

    def _fail_safe(self, reason, sim_time):
        """记录失败原因并进入 SAFE_HOLD，避免继续执行发射动作。"""
        if self.state == SAFE_HOLD:
            return
        self.failure_reason = str(reason)
        if self.verbose:
            print(f"[USER] SAFE_HOLD: {self.failure_reason}")
        self._transition(SAFE_HOLD, sim_time)


__all__ = ["BALL_INITIAL_CONDITIONS", "GO2_INITIAL_STATE", "Go2UserController"]
