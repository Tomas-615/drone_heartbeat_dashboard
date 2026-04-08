import streamlit as st
import pandas as pd
import plotly.express as px
from datetime import datetime, timedelta
import random
import json
import pytz
import folium
from folium.plugins import MarkerCluster, Draw
from streamlit_folium import st_folium

# ===================== 配置与初始化 =====================
st.set_page_config(page_title="无人机任务系统", layout="wide")
beijing_tz = pytz.timezone("Asia/Shanghai")

DRONE_ID = "UAV-007"
HEARTBEAT_INTERVAL = 2  # 秒
OFFLINE_THRESHOLD = 5   # 秒

# 会话状态初始化
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "last_heartbeat" not in st.session_state:
    st.session_state.last_heartbeat = None
if "drone_status" not in st.session_state:
    st.session_state.drone_status = "未知"
if "point_a" not in st.session_state:
    st.session_state.point_a = None
if "point_b" not in st.session_state:
    st.session_state.point_b = None
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "GCJ-02"
if "obstacle_zones" not in st.session_state:
    st.session_state.obstacle_zones = []

# ===================== 工具函数 =====================
def wgs84_to_gcj02(lat, lon):
    import math
    a = 6378245.0
    ee = 0.00669342162296594323
    pi = 3.14159265358979323846
    dLat = 3000.0 + lat + 0.1 * lon
    dLon = 105.0 + lon + 0.1 * lat
    radLat = dLat * pi / 180.0
    magic = math.sin(radLat)
    magic = 1 - ee * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = ((lat - 35.0) * 180.0) / (a * (1 - ee) / (magic * sqrtMagic) * pi)
    dLon = ((lon - 105.0) * 180.0) / (a / sqrtMagic * math.cos(radLat) * pi)
    mgLat = lat + dLat
    mgLon = lon + dLon
    return mgLat, mgLon

def generate_heartbeat():
    now = datetime.now(beijing_tz)
    lat = round(30.6 + random.uniform(-0.01, 0.01), 6)
    lon = round(104.0 + random.uniform(-0.01, 0.01), 6)
    if st.session_state.coord_system == "GCJ-02":
        lat, lon = wgs84_to_gcj02(lat, lon)
    return {
        "timestamp": now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "drone_id": DRONE_ID,
        "status_code": random.choice([200, 200, 200, 500]),
        "latitude": lat,
        "longitude": lon,
        "battery": random.randint(20, 100)
    }

def check_offline():
    if not st.session_state.last_heartbeat:
        return "数据不足"
    now = datetime.now(beijing_tz)
    last_time = datetime.strptime(st.session_state.last_heartbeat, "%Y-%m-%d %H:%M:%S.%f")
    last_time = beijing_tz.localize(last_time)
    delta = (now - last_time).total_seconds()
    if delta >= OFFLINE_THRESHOLD:
        return "掉线"
    else:
        return "在线"

def update_data():
    hb = generate_heartbeat()
    st.session_state.heartbeat_history.append(hb)
    if len(st.session_state.heartbeat_history) > 50:
        st.session_state.heartbeat_history.pop(0)
    st.session_state.last_heartbeat = hb["timestamp"]
    st.session_state.drone_status = check_offline()

def process_draw_data(draw_output):
    if not draw_output:
        return
    if "all_drawings" not in draw_output:
        return
    drawings = draw_output["all_drawings"]
    if not drawings:
        return

    for drawing in drawings:
        if drawing["geometry"]["type"] == "Polygon":
            coordinates = drawing["geometry"]["coordinates"][0]
            polygon_coords = [(coord[1], coord[0]) for coord in coordinates]
            if polygon_coords not in st.session_state.obstacle_zones:
                st.session_state.obstacle_zones.append(polygon_coords)

# ===================== 导航菜单 =====================
menu = st.sidebar.selectbox(
    "功能页面",
    ["航线规划", "飞行监控"]
)

with st.sidebar.expander("坐标系设置"):
    coord = st.radio(
        "输入坐标系",
        ["WGS-84", "GCJ-02(高德/百度)"],
        index=1 if st.session_state.coord_system == "GCJ-02" else 0
    )
    st.session_state.coord_system = "GCJ-02" if "GCJ" in coord else "WGS-84"

st.sidebar.markdown("---")
st.sidebar.subheader("系统状态")
st.sidebar.success("A点已设" if st.session_state.point_a else "A点未设")
st.sidebar.success("B点已设" if st.session_state.point_b else "B点未设")
st.sidebar.warning(f"已标记障碍物区域：{len(st.session_state.obstacle_zones)} 个")

# ===================== 页面1：航线规划 =====================
if menu == "航线规划":
    st.title("🗺️ 航线规划 - 校园地图")
    col1, col2 = st.columns([3, 1])

    with col1:
        center_lat, center_lon = 32.2422, 118.7490
        if st.session_state.coord_system == "GCJ-02":
            center_lat, center_lon = wgs84_to_gcj02(center_lat, center_lon)

        # ========== 地图优化：无限放大 + 自动2D ==========
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=18,
            min_zoom=10,
            max_zoom=22,          # 允许超大放大
            tiles="cartodb positron",  # 2D简洁底图
            prefer_canvas=True,   # 2D渲染
            zoom_control=True,
            scrollWheelZoom=True,
            dragging=True
        )

        # 卫星图（到18级）
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='卫星图',
            name='卫星地图',
            max_zoom=18
        ).add_to(m)

        # 2D无限放大图层（永远不空白）
        folium.TileLayer(
            tiles='https://cartodb-basemaps-{s}.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png',
            attr='2D平面图',
            name='2D平面地图',
            max_zoom=22
        ).add_to(m)

        folium.LayerControl().add_to(m)

        # A/B点
        if st.session_state.point_a:
            folium.Marker(location=st.session_state.point_a, popup="起点A", icon=folium.Icon(color="red")).add_to(m)
        if st.session_state.point_b:
            folium.Marker(location=st.session_state.point_b, popup="终点B", icon=folium.Icon(color="green")).add_to(m)
            folium.PolyLine([st.session_state.point_a, st.session_state.point_b], color="blue", weight=3).add_to(m)

        # 障碍物
        for zone in st.session_state.obstacle_zones:
            folium.Polygon(
                locations=zone,
                popup="障碍物区域",
                color="red",
                fill_color="red",
                fill_opacity=0.3,
                weight=2
            ).add_to(m)

        # 绘制工具
        draw = Draw(
            draw_options={"polyline": False, "polygon": True, "circle": False, "rectangle": False, "marker": False},
            edit_options={"edit": False}
        )
        draw.add_to(m)

        draw_output = st_folium(m, width=900, height=600)
        process_draw_data(draw_output)

    with col2:
        st.subheader("控制面板")
        st.markdown("#### 起点 A")
        a_lat = st.number_input("纬度", value=center_lat, format="%.6f", key="a_lat")
        a_lon = st.number_input("经度", value=center_lon, format="%.6f", key="a_lon")
        if st.button("设置 A 点"):
            st.session_state.point_a = (a_lat, a_lon)
            st.success("A点已设置")

        st.markdown("#### 终点 B")
        b_lat = st.number_input("纬度", value=center_lat+0.001, format="%.6f", key="b_lat")
        b_lon = st.number_input("经度", value=center_lon+0.001, format="%.6f", key="b_lon")
        if st.button("设置 B 点"):
            st.session_state.point_b = (b_lat, b_lon)
            st.success("B点已设置")

        flight_alt = st.slider("设定飞行高度(m)", 10, 100, 30)
        st.info(f"当前设定高度: {flight_alt} m")

        st.markdown("---")
        st.subheader("障碍物区域管理")
        if st.button("保存当前绘制的障碍物"):
            if draw_output and "all_drawings" in draw_output and draw_output["all_drawings"]:
                st.success(f"成功保存 {len(draw_output['all_drawings'])} 个障碍物区域")
                st.rerun()
            else:
                st.warning("暂无新绘制的障碍物区域")
        
        if st.button("清除所有障碍物区域"):
            st.session_state.obstacle_zones = []
            st.success("已清除所有障碍物区域")
            st.rerun()
        
        if st.session_state.obstacle_zones:
            with st.expander("查看障碍物坐标"):
                for i, zone in enumerate(st.session_state.obstacle_zones):
                    st.write(f"障碍物 {i+1}：{zone}")

# ===================== 页面2：飞行监控 =====================
elif menu == "飞行监控":
    st.title("📡 飞行监控 - 无人机心跳监测")
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("模拟一次心跳"):
            update_data()
            st.rerun()
    with col_btn2:
        if st.button("重置数据"):
            st.session_state.heartbeat_history = []
            st.session_state.last_heartbeat = None
            st.session_state.drone_status = "未知"
            st.rerun()

    col1, col2, col3 = st.columns(3)
    status_color = "green" if st.session_state.drone_status == "在线" else "red" if st.session_state.drone_status == "掉线" else "gray"
    col1.metric("无人机状态", st.session_state.drone_status, delta_color=status_color)
    col2.metric("最后心跳时间", st.session_state.last_heartbeat or "无数据")
    col3.metric("已记录心跳数", len(st.session_state.heartbeat_history))

    tab1, tab2, tab3 = st.tabs(["📈 心跳间隔趋势", "📊 原始数据", "⚙️ 检测逻辑"])
    with tab1:
        if len(st.session_state.heartbeat_history) >= 2:
            df = pd.DataFrame(st.session_state.heartbeat_history)
            df["timestamp_dt"] = pd.to_datetime(df["timestamp"])
            df["interval_s"] = df["timestamp_dt"].diff().dt.total_seconds()
            fig = px.line(df, x="timestamp", y="interval_s", title="心跳间隔变化曲线")
            fig.add_hline(y=OFFLINE_THRESHOLD, line_dash="dash", line_color="red", annotation_text="掉线阈值")
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("数据不足，无法绘制趋势图")

    with tab2:
        if st.session_state.heartbeat_history:
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_history), use_container_width=True)
        else:
            st.warning("暂无数据")

    with tab3:
        st.code("""
def check_offline():
    if not st.session_state.last_heartbeat:
        return "数据不足"
    now = datetime.now(beijing_tz)
    last_time = datetime.strptime(st.session_state.last_heartbeat, "%Y-%m-%d %H:%M:%S.%f")
    last_time = beijing_tz.localize(last_time)
    delta = (now - last_time).total_seconds()
    if delta >= OFFLINE_THRESHOLD:
        return "掉线"
    else:
        return "在线"
        """)

# 初始化数据
if not st.session_state.heartbeat_history:
    for _ in range(10):
        update_data()
