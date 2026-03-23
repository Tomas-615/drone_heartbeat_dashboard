"""
无人机通信‘心跳’监测可视化系统
功能：模拟心跳数据生成、掉线检测、实时可视化
"""
import streamlit as st
import pandas as pd
import plotly.express as px
from datetime import datetime, timedelta, timezone
import time
import random
import json

# 定义北京时间时区 (UTC+8)
beijing_tz = timezone(timedelta(hours=8))

# 初始化Session State，用于在Streamlit应用中保持数据状态
if 'heartbeat_history' not in st.session_state:
    st.session_state.heartbeat_history = []

if 'drone_status' not in st.session_state:
    st.session_state.drone_status = "在线"
    # 初始化为北京时间
    st.session_state.last_heartbeat_time = datetime.now(beijing_tz)

# 模拟的无人机ID
DRONE_ID = "UAV-007"
HEARTBEAT_INTERVAL = 2  # 正常心跳间隔（秒）
OFFLINE_THRESHOLD = 5   # 掉线判定阈值（秒）

def generate_heartbeat():
    """生成一条模拟心跳数据 (使用北京时间)"""
    # 使用北京时间
    timestamp = datetime.now(beijing_tz)
    # 模拟心跳数据，包含时间、ID、状态码、随机位置和电量
    heartbeat_data = {
        "timestamp": timestamp,
        "drone_id": DRONE_ID,
        "status_code": random.choice([200, 200, 200, 200, 500]),  # 大部分正常，偶尔异常
        "latitude": round(30.67 + random.uniform(-0.01, 0.01), 6),
        "longitude": round(104.06 + random.uniform(-0.01, 0.01), 6),
        "battery": round(random.uniform(20.0, 100.0), 1)
    }
    return heartbeat_data

def check_offline(history):
    """检查无人机是否掉线 (基于北京时间)"""
    if not history:
        return "数据不足", 0
    latest_entry = history[-1]
    # 使用北京时间计算时间差
    time_diff = (datetime.now(beijing_tz) - latest_entry['timestamp']).total_seconds()
    status = "在线" if time_diff < OFFLINE_THRESHOLD else "掉线"
    return status, time_diff

def update_data():
    """模拟更新一次数据：生成新心跳点并添加到历史记录"""
    new_heartbeat = generate_heartbeat()
    st.session_state.heartbeat_history.append(new_heartbeat)
    # 保持历史记录长度，例如只保留最近50条
    if len(st.session_state.heartbeat_history) > 50:
        st.session_state.heartbeat_history.pop(0)
    # 更新状态
    st.session_state.drone_status, time_diff = check_offline(st.session_state.heartbeat_history)
    st.session_state.last_heartbeat_time = st.session_state.heartbeat_history[-1]['timestamp']

# -------------------- Streamlit 应用界面 --------------------
st.set_page_config(page_title="无人机心跳监测面板", layout="wide")
st.title("✈️ 无人机通信‘心跳’监测可视化系统")
st.caption("模拟无人机心跳包数据生成、掉线检测与实时可视化")

# 侧边栏 - 控制面板
with st.sidebar:
    st.header("控制面板")
    if st.button("🚀 模拟一次心跳", use_container_width=True, type="primary"):
        update_data()
        st.rerun()  # 触发界面更新
    if st.button("🔄 重置数据", use_container_width=True):
        st.session_state.heartbeat_history = []
        st.session_state.drone_status = "在线"
        st.rerun()

    st.divider()
    st.subheader("系统参数")
    st.write(f"**无人机ID:** {DRONE_ID}")
    st.write(f"**正常心跳间隔:** {HEARTBEAT_INTERVAL} 秒")
    st.write(f"**掉线判定阈值:** {OFFLINE_THRESHOLD} 秒")
    st.divider()
    st.caption("**使用说明:** 点击左侧按钮模拟心跳数据。当最后一条数据与当前时间差超过阈值时，状态变为'掉线'。")

# 主显示区域
col1, col2, col3 = st.columns(3)
with col1:
    # 状态指示器
    status_color = "🟢" if st.session_state.drone_status == "在线" else "🔴"
    st.metric("无人机状态", f"{status_color} {st.session_state.drone_status}")
with col2:
    # 最后心跳时间
    last_time_str = st.session_state.last_heartbeat_time.strftime("%H:%M:%S") if st.session_state.heartbeat_history else "N/A"
    st.metric("最后心跳时间", last_time_str)
with col3:
    # 历史数据量
    st.metric("已记录心跳数", len(st.session_state.heartbeat_history))

st.divider()

# 数据显示区域
tab1, tab2, tab3 = st.tabs(["📈 心跳间隔趋势", "📊 原始数据", "⚙️ 检测逻辑与配置"])

with tab1:
    st.subheader("心跳间隔变化趋势图")
    if len(st.session_state.heartbeat_history) > 1:
        # 准备图表数据：计算每次心跳的时间间隔
        df_plot = pd.DataFrame(st.session_state.heartbeat_history)
        df_plot['time_diff'] = df_plot['timestamp'].diff().dt.total_seconds().fillna(0)
        df_plot['index'] = range(len(df_plot))

        # 使用Plotly绘制折线图
        fig = px.line(df_plot, x='index', y='time_diff',
                      title=f'无人机 [{DRONE_ID}] 心跳间隔 (秒)',
                      labels={'index': '心跳序列号', 'time_diff': '间隔时间 (秒)'},
                      markers=True)
        # 添加掉线阈值参考线
        fig.add_hline(y=OFFLINE_THRESHOLD, line_dash="dash", line_color="red",
                      annotation_text=f"掉线阈值 ({OFFLINE_THRESHOLD}秒)",
                      annotation_position="bottom right")
        fig.update_layout(height=400)
        st.plotly_chart(fig, use_container_width=True)
        st.caption("图表说明：每个点代表相邻两次心跳的时间间隔。若间隔超过红色虚线(阈值)，则判断为掉线。")
    else:
        st.info("数据不足，请点击左侧‘模拟一次心跳’按钮生成数据。")

with tab2:
    st.subheader("心跳数据记录表")
    if st.session_state.heartbeat_history:
        df_table = pd.DataFrame(st.session_state.heartbeat_history)
        # 格式化时间显示 (包含时区信息)
        df_table['timestamp'] = df_table['timestamp'].dt.strftime('%H:%M:%S.%f')[:-3]
        st.dataframe(df_table, use_container_width=True, hide_index=True)
        # 显示最新一条数据的JSON格式
        st.json(st.session_state.heartbeat_history[-1], expanded=False)
    else:
        st.info("暂无数据。")

with tab3:
    st.subheader("掉线检测核心逻辑")
    code = '''
def check_offline(history):
    """检查无人机是否掉线 (基于北京时间)"""
    if not history:
        return "数据不足", 0
    latest_entry = history[-1]
    # 使用北京时间计算时间差
    time_diff = (datetime.now(beijing_tz) - latest_entry['timestamp']).total_seconds()
    # 判断：时间差超过阈值则为掉线
    status = "在线" if time_diff < OFFLINE_THRESHOLD else "掉线"
    return status, time_diff
    '''
    st.code(code, language='python')
    st.markdown(f"""
    **关键参数：**
    - **OFFLINE_THRESHOLD** = `{OFFLINE_THRESHOLD}` 秒
    - 当前系统时间 (北京时间): `{datetime.now(beijing_tz).strftime('%H:%M:%S')}`
    - 最后心跳时间: `{st.session_state.last_heartbeat_time.strftime('%H:%M:%S') if st.session_state.heartbeat_history else 'N/A'}`
    - 时间差: `{(datetime.now(beijing_tz) - st.session_state.last_heartbeat_time).total_seconds() if st.session_state.heartbeat_history else 0:.2f}` 秒
    """)

# 页脚
st.divider()
st.caption("""
**项目说明**：这是一个模拟系统，用于演示无人机心跳通信的可视化监测。
- **心跳数据**：每点击一次按钮，模拟生成一条包含时间、位置、状态、电量的数据。
- **掉线检测**：基于最后一条数据的时间与当前时间的差值进行判断。
- **可视化**：实时展示心跳间隔趋势与原始数据。
- **时区**：系统时间已明确设置为北京时间 (UTC+8)。
""")

# 本地直接运行时的入口
if __name__ == "__main__":
    # 如果在本地运行，可以预先添加一些模拟数据以便展示 (使用北京时间)
    if not st.session_state.heartbeat_history:
        for _ in range(10):
            # 生成一些历史数据，时间设置为过去的时刻 (使用北京时间)
            fake_time = datetime.now(beijing_tz) - timedelta(seconds=random.uniform(0, 20))
            fake_data = {
                "timestamp": fake_time,
                "drone_id": DRONE_ID,
                "status_code": 200,
                "latitude": round(30.67 + random.uniform(-0.01, 0.01), 6),
                "longitude": round(104.06 + random.uniform(-0.01, 0.01), 6),
                "battery": round(random.uniform(20.0, 100.0), 1)
            }
            st.session_state.heartbeat_history.append(fake_data)
        st.session_state.heartbeat_history.sort(key=lambda x: x['timestamp'])
    # 注意：Streamlit应用在脚本运行时已自动进入循环，无需手动调用