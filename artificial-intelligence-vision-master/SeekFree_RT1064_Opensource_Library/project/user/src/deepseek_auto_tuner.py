import os
# 代理设置（如果不需要可以注释）
os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7897'
os.environ['HTTPS_PROXY'] = 'http://127.0.0.1:7897'

import serial
import time
import json
from openai import OpenAI

# ================= 1. 核心配置区 =================
COM_PORT = 'COM5'      # 你的串口号
BAUD_RATE = 115200

# 【必改】填入你的 DeepSeek API Key
API_KEY = "sk-bfc254c9394145fbb24f87619b2de0c8"

TARGET_SPEED = 0.4     # 测试的目标速度
TEST_DURATION = 2.0    # 每次测试电机运行 2 秒

# 初始化 DeepSeek 客户端（兼容 OpenAI 接口）
client = OpenAI(
    api_key=API_KEY,
    base_url="https://api.deepseek.com/v1"
)

response = client.chat.completions.create(
    model="deepseek-chat",                    # 这个就是 V3.2
    messages=[
        {"role": "system", "content": "你是一个 PID 整定专家"},
        {"role": "user", "content": "当前 Kp=80, Ki=0.8，速度数据是 [...]"}
    ],
    temperature=0.3,
    max_tokens=300
)

# ================= 2. 串口初始化 =================
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    print(f"✅ 成功接管单片机串口 {COM_PORT}")
except Exception as e:
    print(f"❌ 串口连接失败: {e}")
    exit()

# 初始参数（根据你的电机特性可调整）
current_kp = 80.0
current_ki = 0.8

def send_command(cmd_str):
    """向下位机发送指令"""
    ser.write((cmd_str + "\n").encode('utf-8'))
    time.sleep(0.05)

def run_step_response_test(kp, ki):
    """自动化测试流程"""
    print(f"\n▶️ 开始阶跃响应测试 (当前参数: Kp={kp:.1f}, Ki={ki:.1f})")

    send_command("target:0")
    time.sleep(1.0)
    ser.reset_input_buffer()

    send_command(f"kp:{kp}")
    send_command(f"ki:{ki}")

    speed_data = []
    send_command(f"target:{TARGET_SPEED}")
    start_time = time.time()

    while time.time() - start_time < TEST_DURATION:
        try:
            line = ser.readline().decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) == 6:
                actual_speed = float(parts[1])
                speed_data.append(actual_speed)
        except Exception:
            pass

    send_command("target:0")
    print(f"⏹️ 测试完毕，共采集到 {len(speed_data)} 个速度采样点。")
    return speed_data[::2]   # 降采样，减少数据量

def ask_deepseek_for_new_params(speed_data, kp, ki):
    """调用 DeepSeek 分析波形并返回新参数"""
    print("🧠 [DeepSeek 思考中] 正在分析波形...")

    prompt = f"""
你是一位资深的嵌入式控制算法工程师。我正在调试一个直流减速电机的增量式 PI 速度环。
控制器运行频率为 50Hz (20ms周期)。目标速度设定为 {TARGET_SPEED} m/s。
当前使用的参数是 Kp={kp}, Ki={ki}。

以下是电机在阶跃指令后 2 秒内记录的实际速度时序数组：
{speed_data}

请推理下一步的整定策略，并输出新的 Kp 和 Ki。
【严格格式要求】：你必须且只能返回一段合法的 JSON 文本。
JSON 格式范例：
{{"Kp": 150.0, "Ki": 5.0, "reason": "波形上升过慢，需要增加Kp以提高响应速度。"}}
"""

    try:
        response = client.chat.completions.create(
            model="deepseek-chat",
            messages=[
                {"role": "system", "content": "你是一个专业的 PID 整定专家，只输出 JSON 格式的结果。"},
                {"role": "user", "content": prompt}
            ],
            temperature=0.3,
            max_tokens=300,
            timeout=30
        )

        text = response.choices[0].message.content.strip()
        # 去除可能存在的 markdown 标记
        if text.startswith("```json"):
            text = text[7:]
        if text.startswith("```"):
            text = text[3:]
        if text.endswith("```"):
            text = text[:-3]

        result = json.loads(text)
        return result['Kp'], result['Ki'], result.get('reason', '无详细说明')
    except Exception as e:
        print(f"❌ DeepSeek API 调用失败: {e}")
        return kp, ki, "API 调用异常"

# ================= 3. 主循环：自动化迭代 =================
print("\n🏁 === DeepSeek 自动 PID 调参系统启动 ===")
try:
    for iteration in range(1, 11):
        print(f"\n\n================= [ 第 {iteration}/10 代进化 ] =================")

        speed_array = run_step_response_test(current_kp, current_ki)

        if len(speed_array) < 5:
            print("⚠️ 警告：采集到的数据太少，请检查单片机是否正常发送数据！")
            time.sleep(2)
            continue

        new_kp, new_ki, reason = ask_deepseek_for_new_params(speed_array, current_kp, current_ki)

        if reason == "API 调用异常":
            print("🤖 [系统保护]: API 调用失败，强制进入 15 秒冷却...")
            time.sleep(15)
        else:
            print(f"🤖 [DeepSeek 诊断结论]: {reason}")
            print(f"✨ [参数进化]: (Kp:{current_kp}->{new_kp}), (Ki:{current_ki}->{new_ki})")
            current_kp = float(new_kp)
            current_ki = float(new_ki)
            print("⏳ 冷却 5 秒后进入下一轮...")
            time.sleep(5)

except KeyboardInterrupt:
    print("\n\n🛑 用户手动终止了自动调参。")
finally:
    send_command("target:0")
    ser.close()
    print(f"\n🎉 调参结束，请将最终收敛的参数写死在单片机代码里: Kp={current_kp}, Ki={current_ki}")