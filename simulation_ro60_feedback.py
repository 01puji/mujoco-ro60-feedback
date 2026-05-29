import time
import math
import re
import serial
import mujoco
import mujoco.viewer


# ==============================
# 1. Serial setting
# ==============================

PORT = "COM5"
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
time.sleep(2)
ser.reset_input_buffer()

print(f"Connected to Arduino Mega on {PORT}")


# ==============================
# 2. Load MuJoCo model
# ==============================

model = mujoco.MjModel.from_xml_path("leg_1dof.xml")
data = mujoco.MjData(model)

joint_id = model.joint("hip_joint").id
qpos_addr = model.jnt_qposadr[joint_id]
site_id = model.site("leg_end").id

print(f"joint_id={joint_id}, qpos_addr={qpos_addr}")


# ==============================
# 3. Calibration setting
# ==============================

# 第一次读取到的编码器角度作为零点
zero_set = False
zero_angle = 0.0

# 如果虚拟腿方向与真实方向相反，改成 -1.0
DIRECTION = 1.0

# 角度比例校正
# 你现在实测：实际旋转约 90°，代码显示约 65°
# 所以需要放大：90 / 65 = 1.3846
# 原比例是 1/3，因此新比例约为 0.4615
ANGLE_SCALE = 0.4615

# 现在模型中 0° 已经是向下，所以初始角度为 0
INITIAL_LEG_ANGLE = 0.0


# ==============================
# 4. Functions
# ==============================

def parse_angle(line):
    """
    从串口文本中提取第一个数字。
    支持：
    12.35
    Angle: 12.35 deg
    """
    match = re.search(r"[-+]?\d*\.\d+|[-+]?\d+", line)

    if match:
        return float(match.group())

    return None


def wrap_angle_deg(angle):
    """
    限制角度到 -180 ~ 180，避免持续累加过大。
    """
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360

    return angle


# ==============================
# 5. Set initial pose
# ==============================

data.qpos[qpos_addr] = math.radians(INITIAL_LEG_ANGLE)
data.qvel[qpos_addr] = 0.0
mujoco.mj_forward(model, data)


# ==============================
# 6. Main loop
# ==============================

last_print = 0

angle_raw = 0.0
angle_relative = 0.0
angle_virtual = INITIAL_LEG_ANGLE

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():

        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if line:
            angle = parse_angle(line)

            if angle is not None:
                angle_raw = angle

                if not zero_set:
                    zero_angle = angle_raw
                    zero_set = True
                    print(f"Zero angle set: {zero_angle:.2f} deg")

                # 编码器相对初始角度
                angle_relative = angle_raw - zero_angle

                # 现在 0° 就是向下
                angle_virtual = INITIAL_LEG_ANGLE + DIRECTION * angle_relative * ANGLE_SCALE

                angle_virtual = wrap_angle_deg(angle_virtual)

                with viewer.lock():
                    data.qpos[qpos_addr] = math.radians(angle_virtual)
                    data.qvel[qpos_addr] = 0.0
                    mujoco.mj_forward(model, data)

        viewer.sync()

        if time.time() - last_print > 0.3:
            qpos_angle = math.degrees(data.qpos[qpos_addr])
            end_pos = data.site_xpos[site_id]

            print(
                f"encoder_raw={angle_raw:.2f} deg, "
                f"relative={angle_relative:.2f} deg, "
                f"leg_angle={angle_virtual:.2f} deg, "
                f"qpos={qpos_angle:.2f} deg, "
                f"end=({end_pos[0]:.3f}, {end_pos[1]:.3f})"
            )

            last_print = time.time()

        time.sleep(0.01)

ser.close()