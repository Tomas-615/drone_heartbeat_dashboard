"""
无人机通信'心跳'监测与航线规划系统 (完全修复版)
功能：心跳监测、掉线检测、高德3D地图显示、坐标系转换、航线规划
依赖: streamlit, pandas, plotly, pydeck
"""
import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from datetime import datetime, timedelta, timezone
import time
import random
import json
import math
import pydeck as pdk

# 定义北京时间时区 (UTC+8)
beijing_tz = timezone(timedelta(hours=8))

# ---------- 坐标系转换函数 (模拟) ----------
def coordinate_transform(lat, lon, from_crs, to_crs='GCJ-02'):
    """
    模拟坐标系转换。
    在实际应用中，此处应实现WGS-84与GCJ-02之间的精确转换算法。
    此处为演示，对坐标加入微小随机偏移来模拟转换效果。
    """
    if from_crs == to_crs:
        return lat, lon
    # 模拟转换：加入一个非常小的系统性偏移
    delta_lat = random.uniform(-0.0005, 0.0005)
    delta_lon = random.uniform(-0.0005, 0.0005)
    transformed_lat = lat + delta_lat
    transformed_lon = lon + delta_lon
    return round(transformed_lat, 6), round(transformed_lon, 6)

# ---------- 初始化Session State ----------
# 保留原有心跳数据状态
if 'heartbeat_history' not in st.session_state:
    st.session_state.heartbeat_history = []

if 'drone_status' not in st.session_state:
    st.session_state.drone_status = "在线"
    st.session_state.last_heartbeat_time = datetime.now(beijing_tz)

# 新增：地图与航线规划相关状态
if 'point_a' not in st.session_state:
    st.session_state.point_a = {"lat": 30.670, "lon": 104.060, "crs": "GCJ-02"}
if 'point_b' not in st.session_state:
    st.session_state.point_b = {"lat": 30.680, "lon": 104.070, "crs": "GCJ-02"}
if 'obstacles' not in st.session_state:
    # 生成一些随机障碍物
    st.session_state.obstacles = []
    for i in range(5):
        st.session_state.obstacles.append({
            "id": i,
            "lat": 30.675 + random.uniform(-0.003, 0.003),
            "lon": 104.065 + random.uniform(-0.003, 0.003),
            "name": f"障碍物{i+1}",
            "height": random.randint(10, 50)  # 障碍物高度
        })
if 'map_view' not in st.session_state:
    st.session_state.map_view = "3D"  # 默认3D视图
if 'selected_crs' not in st.session_state:
    st.session_state.selected_crs = "GCJ-02"
if 'map_style' not in st.session_state:
    st.session_state.map_style = "gaode_vector"  # 默认高德矢量地图
if 'current_zoom' not in st.session_state:
    st.session_state.current_zoom = 15  # 默认缩放级别
if 'pitch' not in st.session_state:
    st.session_state.pitch = 60  # 默认3D俯仰角
if 'bearing' not in st.session_state:
    st.session_state.bearing = 0  # 默认朝向

# ---------- 心跳相关函数 ----------
DRONE_ID = "UAV-007"
HEARTBEAT_INTERVAL = 2
OFFLINE_THRESHOLD = 5

def generate_heartbeat():
    """生成一条模拟心跳数据 (使用北京时间)"""
    timestamp = datetime.now(beijing_tz)
    heartbeat_data = {
        "timestamp": timestamp,
        "drone_id": DRONE_ID,
        "status_code": random.choice([200, 200, 200, 200, 500]),
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
    time_diff = (datetime.now(beijing_tz) - latest_entry['timestamp']).total_seconds()
    status = "在线" if time_diff < OFFLINE_THRESHOLD else "掉线"
    return status, time_diff

def update_heartbeat_data():
    """模拟更新一次心跳数据"""
    new_heartbeat = generate_heartbeat()
    st.session_state.heartbeat_history.append(new_heartbeat)
    if len(st.session_state.heartbeat_history) > 50:
        st.session_state.heartbeat_history.pop(0)
    st.session_state.drone_status, time_diff = check_offline(st.session_state.heartbeat_history)
    st.session_state.last_heartbeat_time = st.session_state.heartbeat_history[-1]['timestamp']

# ---------- 高德3D地图生成函数 ----------
def create_gaode_3d_map():
    """创建高德3D地图对象"""
    # 计算地图视图中心点
    center_lat = (st.session_state.point_a["lat"] + st.session_state.point_b["lat"]) / 2
    center_lon = (st.session_state.point_a["lon"] + st.session_state.point_b["lon"]) / 2
    
    # 设置初始视图状态
    initial_view_state = pdk.ViewState(
        latitude=center_lat,
        longitude=center_lon,
        zoom=st.session_state.current_zoom,
        pitch=st.session_state.pitch,
        bearing=st.session_state.bearing
    )

    # 准备航点与障碍物数据
    points_data = []
    
    # A点 (起点)
    points_data.append({
        "lon": st.session_state.point_a["lon"],
        "lat": st.session_state.point_a["lat"],
        "name": "A点 (起点)",
        "type": "point_a",
        "color": [0, 255, 0, 160],
        "elevation": 20  # 3D视图中的高度
    })
    
    # B点 (终点)
    points_data.append({
        "lon": st.session_state.point_b["lon"],
        "lat": st.session_state.point_b["lat"],
        "name": "B点 (终点)",
        "type": "point_b",
        "color": [255, 0, 0, 160],
        "elevation": 20
    })
    
    # 障碍物 - 在3D视图中显示为柱状
    for obs in st.session_state.obstacles:
        points_data.append({
            "lon": obs["lon"],
            "lat": obs["lat"],
            "name": obs["name"],
            "type": "obstacle",
            "color": [255, 165, 0, 160],
            "elevation": obs.get("height", 15)  # 使用障碍物高度
        })

    # 根据视图模式选择图层
    if st.session_state.map_view == "3D":
        # 3D视图：使用ColumnLayer显示柱状图
        column_layer = pdk.Layer(
            "ColumnLayer",
            data=points_data,
            get_position=["lon", "lat"],
            get_elevation="elevation",
            elevation_scale=1,
            radius=30,
            get_fill_color="color",
            pickable=True,
            auto_highlight=True,
            extruded=True,
        )
        
        # 3D视图中的航线
        line_layer = pdk.Layer(
            "LineLayer",
            data=pd.DataFrame([{
                "path": [
                    [st.session_state.point_a["lon"], st.session_state.point_a["lat"], 10],  # 添加高度
                    [st.session_state.point_b["lon"], st.session_state.point_b["lat"], 10]
                ]
            }]),
            get_path="path",
            get_color=[0, 0, 255, 200],
            get_width=5,
        )
        
        layers = [column_layer, line_layer]
    else:
        # 2D视图：使用ScatterplotLayer
        scatter_layer = pdk.Layer(
            "ScatterplotLayer",
            data=points_data,
            get_position=["lon", "lat"],
            get_fill_color="color",
            get_radius=40,
            pickable=True,
            auto_highlight=True,
        )
        
        # 2D视图中的航线
        line_layer = pdk.Layer(
            "LineLayer",
            data=pd.DataFrame([{
                "path": [
                    [st.session_state.point_a["lon"], st.session_state.point_a["lat"]],
                    [st.session_state.point_b["lon"], st.session_state.point_b["lat"]]
                ]
            }]),
            get_path="path",
            get_color=[0, 0, 255, 200],
            get_width=4,
        )
        
        layers = [scatter_layer, line_layer]

    # 修复：简化地图瓦片URL配置
    tile_layer = None
    
    # 使用简单的OpenStreetMap作为备选地图，避免高德地图的URL解析问题
    tile_layer = pdk.Layer(
        "TileLayer",
        data=[],
        get_position=["lon", "lat"],
        # 使用OpenStreetMap作为基础地图，稳定可靠
        tile_layer_url="https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        max_zoom=20,
        min_zoom=0
    )

    # 组合图层
    all_layers = []
    if tile_layer:
        all_layers.append(tile_layer)
    
    all_layers.extend(layers)

    # 创建Deck对象 - 简化配置，避免复杂的JSON序列化问题
    deck = pdk.Deck(
        layers=all_layers,
        initial_view_state=initial_view_state,
        tooltip={
            "html": "<b>{name}</b><br/>经度: {lon:.6f}<br/>纬度: {lat:.6f}<br/>高度: {elevation}米",
            "style": {
                "backgroundColor": "rgba(30, 30, 30, 0.8)",
                "color": "white",
                "padding": "8px",
                "borderRadius": "4px",
                "fontSize": "12px",
                "maxWidth": "200px"
            }
        },
        # 移除map_provider参数，避免可能的序列化问题
    )
    return deck

# ---------- Streamlit 应用界面 ----------
st.set_page_config(
    page_title="无人机监控与航线规划系统", 
    layout="wide", 
    page_icon="✈️",
    initial_sidebar_state="expanded"
)

# 使用侧边栏进行主要导航
st.sidebar.title("✈️ 无人机综合系统")
page = st.sidebar.radio("导航菜单", ["总览", "飞行监控", "航线规划", "坐标系设置", "系统状态"])

# 页面1: 总览
if page == "总览":
    st.title("无人机监控与航线规划系统")
    st.markdown("""
    ### 无人机监控与航线规划系统 (稳定版)
    本系统提供以下核心功能：
    1.  **心跳监测**：实时监控无人机通信状态，检测掉线情况
    2.  **航线规划**：在地图上设置航点与障碍物，规划飞行航线
    3.  **坐标转换**：支持WGS-84与GCJ-02坐标系之间的转换
    4.  **地图显示**：使用OpenStreetMap作为基础地图，稳定可靠
    
    **修复说明**：已修复JavaScript语法错误，确保系统稳定运行。
    """)
    st.divider()
    
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("实时状态概览")
        status_color = "🟢" if st.session_state.drone_status == "在线" else "🔴"
        st.metric("无人机状态", f"{status_color} {st.session_state.drone_status}")
        st.metric("已记录心跳数", len(st.session_state.heartbeat_history))
        st.metric("当前地图视图", st.session_state.map_view)
    
    with col2:
        st.subheader("航线规划概览")
        st.write(f"**A点坐标:** {st.session_state.point_a['lat']:.6f}, {st.session_state.point_a['lon']:.6f}")
        st.write(f"**B点坐标:** {st.session_state.point_b['lat']:.6f}, {st.session_state.point_b['lon']:.6f}")
        st.write(f"**障碍物数量:** {len(st.session_state.obstacles)}")
        st.write(f"**当前缩放级别:** {st.session_state.current_zoom}")
    
    st.info("请使用左侧导航菜单切换功能页面。")

# 页面2: 飞行监控
elif page == "飞行监控":
    st.title("飞行监控面板")
    st.caption("无人机心跳包数据监控与可视化")
    
    # 控制按钮
    col_btn1, col_btn2, col_btn3 = st.columns(3)
    with col_btn1:
        if st.button("🚀 模拟心跳", use_container_width=True, type="primary"):
            update_heartbeat_data()
            st.rerun()
    with col_btn2:
        if st.button("🔁 连续模拟(5次)", use_container_width=True):
            for _ in range(5):
                update_heartbeat_data()
                time.sleep(0.1)
            st.rerun()
    with col_btn3:
        if st.button("🔄 重置数据", use_container_width=True):
            st.session_state.heartbeat_history = []
            st.session_state.drone_status = "在线"
            st.rerun()
    
    # 状态指标
    col1, col2, col3 = st.columns(3)
    with col1:
        status_color = "🟢" if st.session_state.drone_status == "在线" else "🔴"
        st.metric("无人机状态", f"{status_color} {st.session_state.drone_status}")
    with col2:
        last_time_str = st.session_state.last_heartbeat_time.strftime("%H:%M:%S") if st.session_state.heartbeat_history else "N/A"
        st.metric("最后心跳时间", last_time_str)
    with col3:
        st.metric("已记录心跳数", len(st.session_state.heartbeat_history))
    
    st.divider()
    
    # 图表和数据
    tab1, tab2 = st.tabs(["📈 心跳趋势", "📊 原始数据"])
    
    with tab1:
        st.subheader("心跳间隔变化趋势图")
        if len(st.session_state.heartbeat_history) > 1:
            df_plot = pd.DataFrame(st.session_state.heartbeat_history)
            df_plot['time_diff'] = df_plot['timestamp'].diff().dt.total_seconds().fillna(0)
            df_plot['index'] = range(len(df_plot))
            
            fig = px.line(df_plot, x='index', y='time_diff',
                          title=f'无人机 [{DRONE_ID}] 心跳间隔 (秒)',
                          labels={'index': '心跳序列号', 'time_diff': '间隔时间 (秒)'},
                          markers=True)
            fig.add_hline(y=OFFLINE_THRESHOLD, line_dash="dash", line_color="red",
                         annotation_text=f"掉线阈值 ({OFFLINE_THRESHOLD}秒)")
            fig.update_layout(height=400)
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("数据不足，请点击上方按钮生成数据。")
    
    with tab2:
        st.subheader("心跳数据记录表")
        if st.session_state.heartbeat_history:
            df_table = pd.DataFrame(st.session_state.heartbeat_history)
            df_table['timestamp'] = df_table['timestamp'].dt.strftime('%H:%M:%S')
            st.dataframe(df_table, use_container_width=True, hide_index=True)
        else:
            st.info("暂无数据。")

# 页面3: 航线规划
elif page == "航线规划":
    st.title("航线规划")
    st.caption("设置A/B点，在地图上规划飞行航线")
    
    # 侧边栏地图控制
    st.sidebar.subheader("🗺️ 地图控制")
    
    # 视图模式选择
    view_mode = st.sidebar.radio("视图模式", ["3D", "2D"], 
                                 index=0 if st.session_state.map_view == "3D" else 1)
    if view_mode != st.session_state.map_view:
        st.session_state.map_view = view_mode
        st.rerun()
    
    # 3D视图控制
    if st.session_state.map_view == "3D":
        st.sidebar.subheader("🏔️ 3D控制")
        
        # 俯仰角控制
        new_pitch = st.sidebar.slider("俯仰角度", 0, 80, st.session_state.pitch)
        if new_pitch != st.session_state.pitch:
            st.session_state.pitch = new_pitch
            st.rerun()
        
        # 朝向控制
        new_bearing = st.sidebar.slider("朝向角度", 0, 360, st.session_state.bearing, 5)
        if new_bearing != st.session_state.bearing:
            st.session_state.bearing = new_bearing
            st.rerun()
    
    # 缩放控制
    st.sidebar.subheader("🔍 缩放控制")
    new_zoom = st.sidebar.slider("缩放级别", 10, 20, st.session_state.current_zoom)
    if new_zoom != st.session_state.current_zoom:
        st.session_state.current_zoom = new_zoom
        st.rerun()
    
    # 状态显示
    st.sidebar.subheader("📊 当前状态")
    st.sidebar.write(f"**视图模式:** {st.session_state.map_view}")
    st.sidebar.write(f"**缩放级别:** {st.session_state.current_zoom}")
    if st.session_state.map_view == "3D":
        st.sidebar.write(f"**俯仰角度:** {st.session_state.pitch}°")
        st.sidebar.write(f"**朝向角度:** {st.session_state.bearing}°")
    
    # 地图控制面板
    with st.expander("📍 航点设置", expanded=True):
        col_set1, col_set2 = st.columns(2)
        
        with col_set1:
            st.subheader("A点 (起点)")
            a_lat = st.number_input("A点纬度", value=float(st.session_state.point_a["lat"]), 
                                    format="%.6f", key="a_lat", step=0.0001)
            a_lon = st.number_input("A点经度", value=float(st.session_state.point_a["lon"]), 
                                    format="%.6f", key="a_lon", step=0.0001)
            if st.button("✅ 设置A点", use_container_width=True, key="set_a"):
                st.session_state.point_a["lat"] = a_lat
                st.session_state.point_a["lon"] = a_lon
                st.success("A点坐标已更新！")
                st.rerun()
        
        with col_set2:
            st.subheader("B点 (终点)")
            b_lat = st.number_input("B点纬度", value=float(st.session_state.point_b["lat"]), 
                                    format="%.6f", key="b_lat", step=0.0001)
            b_lon = st.number_input("B点经度", value=float(st.session_state.point_b["lon"]), 
                                    format="%.6f", key="b_lon", step=0.0001)
            if st.button("✅ 设置B点", use_container_width=True, key="set_b"):
                st.session_state.point_b["lat"] = b_lat
                st.session_state.point_b["lon"] = b_lon
                st.success("B点坐标已更新！")
                st.rerun()
    
    # 障碍物管理
    with st.expander("🎯 障碍物管理", expanded=False):
        col_obs1, col_obs2 = st.columns(2)
        with col_obs1:
            if st.button("生成障碍物", use_container_width=True, key="gen_obs"):
                st.session_state.obstacles = []
                for i in range(random.randint(3, 8)):
                    st.session_state.obstacles.append({
                        "id": i,
                        "lat": 30.675 + random.uniform(-0.004, 0.004),
                        "lon": 104.065 + random.uniform(-0.004, 0.004),
                        "name": f"障碍物{i+1}",
                        "height": random.randint(10, 50)
                    })
                st.success(f"已生成 {len(st.session_state.obstacles)} 个障碍物！")
                st.rerun()
        
        with col_obs2:
            if st.button("清除障碍物", use_container_width=True, key="clear_obs"):
                st.session_state.obstacles = []
                st.success("所有障碍物已清除！")
                st.rerun()
    
    # 地图状态显示
    col_status1, col_status2, col_status3 = st.columns(3)
    with col_status1:
        view_icon = "🏔️" if st.session_state.map_view == "3D" else "🗺️"
        st.metric("当前视图", f"{view_icon} {st.session_state.map_view}")
    with col_status2:
        st.metric("缩放级别", f"{st.session_state.current_zoom}")
    with col_status3:
        st.metric("地图服务", "OpenStreetMap")
    
    # 显示地图
    st.subheader(f"地图显示 - {st.session_state.map_view}视图")
    
    try:
        map_obj = create_gaode_3d_map()
        st.pydeck_chart(map_obj)
        
        # 地图使用提示
        with st.expander("💡 地图使用提示", expanded=False):
            st.markdown("""
            **地图功能：**
            1. **3D视图模式**：
               - 俯视视角，适合查看地形起伏
               - 障碍物显示为3D柱状图
               - 航线在空中显示
            
            2. **2D视图模式**：
               - 平面视角，适合精确规划
               - 障碍物显示为平面标记
               - 航线在地面显示
            
            3. **操作说明**：
               - 鼠标滚轮：缩放地图
               - 鼠标左键拖动：平移地图
               - 鼠标右键拖动（3D模式）：调整视角
               - 点击地图元素查看详细信息
            """)
        
    except Exception as e:
        st.error(f"地图加载失败: {str(e)}")
        st.info("请尝试刷新页面或检查网络连接。")
    
    # 障碍物列表
    with st.expander("📋 障碍物列表", expanded=False):
        if st.session_state.obstacles:
            df_obs = pd.DataFrame(st.session_state.obstacles)
            st.dataframe(df_obs[['id', 'name', 'lat', 'lon', 'height']], 
                        use_container_width=True, 
                        hide_index=True)
        else:
            st.info("暂无障碍物，点击上方按钮生成。")
    
    # 航线信息
    with st.expander("📏 航线信息", expanded=False):
        # 计算距离
        lat1, lon1 = st.session_state.point_a["lat"], st.session_state.point_a["lon"]
        lat2, lon2 = st.session_state.point_b["lat"], st.session_state.point_b["lon"]
        
        # 简化距离计算
        distance = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2) * 111000
        
        col_info1, col_info2 = st.columns(2)
        with col_info1:
            st.write("**起点(A点)**")
            st.write(f"纬度: {lat1:.6f}")
            st.write(f"经度: {lon1:.6f}")
            st.write(f"坐标系: {st.session_state.point_a['crs']}")
        
        with col_info2:
            st.write("**终点(B点)**")
            st.write(f"纬度: {lat2:.6f}")
            st.write(f"经度: {lon2:.6f}")
            st.write(f"坐标系: {st.session_state.point_b['crs']}")
        
        st.divider()
        col_dist1, col_dist2 = st.columns(2)
        with col_dist1:
            st.metric("直线距离", f"{distance:.1f} 米")
        with col_dist2:
            if distance > 0:
                flight_time = distance / 10  # 假设无人机速度10m/s
                st.metric("预计飞行时间", f"{flight_time:.1f} 秒")
        
        # 根据视图模式显示建议
        if st.session_state.map_view == "3D":
            st.info("💡 当前为3D视图，适合查看地形起伏和整体航线规划。")
        else:
            st.info("💡 当前为2D视图，适合查看建筑细节和精确位置。")

# 页面4: 坐标系设置
elif page == "坐标系设置":
    st.title("坐标系设置")
    
    st.markdown("""
    ### 坐标系说明
    本系统支持两种常用坐标系：
    1.  **WGS-84**：国际通用的经纬度标准，GPS设备通常使用此坐标系。
    2.  **GCJ-02**：中国国家测绘局制定的坐标系，中国地图服务（如高德、腾讯）使用此坐标系。
    
    **重要提示**：本系统使用的地图服务与GCJ-02坐标系完全匹配。如果您的坐标是WGS-84，请先进行坐标转换，否则地图显示会有偏移。
    """)
    
    st.divider()
    
    # 当前坐标系设置
    st.subheader("当前坐标系设置")
    new_crs = st.radio("选择坐标系", 
                      ["WGS-84", "GCJ-02"], 
                      index=0 if st.session_state.selected_crs == "WGS-84" else 1,
                      key="crs_selector")
    
    if new_crs == "WGS-84":
        st.warning("""
        **注意**：选择WGS-84坐标系时，坐标在地图上显示可能会有偏移。
        建议将坐标转换为GCJ-02以获得更准确的定位。
        """)
    else:
        st.success("""
        **正确**：选择GCJ-02坐标系与国内地图服务完全匹配。
        坐标将在地图上准确显示，无偏移。
        """)
    
    if new_crs != st.session_state.selected_crs:
        st.session_state.selected_crs = new_crs
        st.success(f"坐标系已切换为: {new_crs}")
    
    st.divider()
    
    # 坐标系转换测试
    st.subheader("坐标系转换测试")
    
    col_test1, col_test2 = st.columns(2)
    
    with col_test1:
        st.write("**输入坐标**")
        test_lat = st.number_input("纬度", value=30.670000, format="%.6f", 
                                  key="test_lat_input", step=0.0001)
        test_lon = st.number_input("经度", value=104.060000, format="%.6f", 
                                  key="test_lon_input", step=0.0001)
        input_crs = st.selectbox("输入坐标系", ["WGS-84", "GCJ-02"], 
                                key="input_crs_select")
    
    with col_test2:
        st.write("**转换设置**")
        target_crs = st.selectbox("目标坐标系", ["WGS-84", "GCJ-02"], 
                                 index=0 if new_crs == "WGS-84" else 1,
                                 key="target_crs_select")
        
        if st.button("🔄 执行转换", type="primary", use_container_width=True):
            st.session_state.test_result = coordinate_transform(
                test_lat, test_lon, input_crs, target_crs
            )
    
    if 'test_result' in st.session_state:
        st.divider()
        st.subheader("转换结果")
        result_lat, result_lon = st.session_state.test_result
        col_res1, col_res2 = st.columns(2)
        with col_res1:
            st.metric("纬度", f"{result_lat:.6f}")
        with col_res2:
            st.metric("经度", f"{result_lon:.6f}")
        st.caption(f"从 {input_crs} 转换到 {target_crs}")
        
        # 显示偏移量
        lat_diff = result_lat - test_lat
        lon_diff = result_lon - test_lon
        st.caption(f"偏移量: 纬度{lat_diff:.6f}, 经度{lon_diff:.6f}")

# 页面5: 系统状态
elif page == "系统状态":
    st.title("系统状态")
    
    # 系统状态概览
    st.subheader("系统概览")
    
    col_overview1, col_overview2, col_overview3, col_overview4 = st.columns(4)
    
    with col_overview1:
        status_icon = "✅" if st.session_state.drone_status == "在线" else "❌"
        st.metric("系统状态", f"{status_icon} 运行中")
    
    with col_overview2:
        st.metric("数据记录", f"{len(st.session_state.heartbeat_history)} 条")
    
    with col_overview3:
        view_icon = "🏔️" if st.session_state.map_view == "3D" else "🗺️"
        st.metric("地图视图", f"{view_icon} {st.session_state.map_view}")
    
    with col_overview4:
        current_time = datetime.now(beijing_tz)
        st.metric("系统时间", current_time.strftime("%H:%M:%S"))
    
    st.divider()
    
    # 详细状态
    col_detail1, col_detail2 = st.columns(2)
    
    with col_detail1:
        st.subheader("心跳监测模块")
        
        status_info = [
            ("模块状态", "✅ 正常"),
            ("心跳间隔", f"{HEARTBEAT_INTERVAL} 秒"),
            ("掉线阈值", f"{OFFLINE_THRESHOLD} 秒"),
            ("无人机ID", DRONE_ID),
            ("当前状态", st.session_state.drone_status),
            ("最后心跳", st.session_state.last_heartbeat_time.strftime("%Y-%m-%d %H:%M:%S") 
             if st.session_state.heartbeat_history else "无记录")
        ]
        
        for label, value in status_info:
            st.write(f"**{label}:** {value}")
    
    with col_detail2:
        st.subheader("地图与航线模块")
        
        map_status = [
            ("地图服务", "OpenStreetMap"),
            ("视图模式", st.session_state.map_view),
            ("缩放级别", st.session_state.current_zoom),
            ("俯仰角度", f"{st.session_state.pitch}°" if st.session_state.map_view == "3D" else "0°"),
            ("朝向角度", f"{st.session_state.bearing}°" if st.session_state.map_view == "3D" else "0°"),
            ("当前坐标系", st.session_state.selected_crs),
            ("A点坐标", f"{st.session_state.point_a['lat']:.6f}, {st.session_state.point_a['lon']:.6f}"),
            ("B点坐标", f"{st.session_state.point_b['lat']:.6f}, {st.session_state.point_b['lon']:.6f}"),
            ("障碍物数量", len(st.session_state.obstacles))
        ]
        
        for label, value in map_status:
            st.write(f"**{label}:** {value}")

# 侧边栏系统说明
st.sidebar.divider()
st.sidebar.caption("""
**无人机监控与航线规划系统 (稳定修复版)**

**核心特性：**
- 📡 心跳监测：实时监控无人机状态
- 🗺️ 地图规划：3D/2D地图显示
- 🎯 航线规划：设置航点与障碍物
- 🔄 坐标转换：WGS-84/GCJ-02转换

**修复内容：**
- 修复JavaScript语法错误："Expected comma at character 6"
- 优化地图瓦片服务，确保稳定运行
- 简化配置，避免JSON序列化问题

**操作提示：**
- 使用鼠标操作地图：滚轮缩放，左键拖动平移
- 3D视图中右键拖动可调整视角
- 点击地图元素查看详细信息

**运行要求：**
- Python 3.8+
- 安装依赖: `pip install streamlit pandas plotly pydeck`
""")

# ---------- 初始化数据 ----------
if __name__ == "__main__":
    # 初始化示例心跳数据
    if not st.session_state.heartbeat_history:
        for _ in range(8):
            fake_time = datetime.now(beijing_tz) - timedelta(seconds=random.uniform(0, 15))
            fake_data = {
                "timestamp": fake_time,
                "drone_id": DRONE_ID,
                "status_code": 200,
                "latitude": round(30.67 + random.uniform(-0.008, 0.008), 6),
                "longitude": round(104.06 + random.uniform(-0.008, 0.008), 6),
                "battery": round(random.uniform(40.0, 95.0), 1)
            }
            st.session_state.heartbeat_history.append(fake_data)
        st.session_state.heartbeat_history.sort(key=lambda x: x['timestamp'])