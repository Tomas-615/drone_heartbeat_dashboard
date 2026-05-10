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
import os
import math
from typing import List, Tuple, Dict, Optional
from enum import Enum
import numpy as np
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
from shapely.ops import nearest_points, unary_union
import shapely.affinity
from shapely import buffer

# ===================== 配置与初始化 =====================
st.set_page_config(page_title="无人机任务系统", layout="wide")
beijing_tz = pytz.timezone("Asia/Shanghai")

DRONE_ID = "UAV-007"
HEARTBEAT_INTERVAL = 2  # 秒
OFFLINE_THRESHOLD = 5   # 秒
OBSTACLE_FILE = "obstacle_zones.json"  # 障碍物数据保存文件
OBSTACLE_AVOIDANCE_DISTANCE = 10  # 绕障碍物边缘的距离（米）

# 航线规划模式枚举
class RouteMode(Enum):
    LEFT = "向左绕飞"
    RIGHT = "向右绕飞"
    OPTIMAL = "最优路径（最短弧线）"

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

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两点之间的距离（米）- 使用Haversine公式"""
    R = 6371000  # 地球半径（米）
    
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    distance = R * c
    return distance

def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算从点1到点2的方位角（度）"""
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    dlon = lon2_rad - lon1_rad
    
    x = math.sin(dlon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360  # 确保角度在0-360度之间

def calculate_destination_point(lat: float, lon: float, bearing: float, distance: float) -> Tuple[float, float]:
    """计算从给定点出发，沿指定方位角移动指定距离后的点"""
    R = 6371000  # 地球半径（米）
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)
    
    angular_distance = distance / R
    
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(angular_distance) + 
                         math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad))
    
    lon2_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
                                    math.cos(angular_distance) - math.sin(lat_rad) * math.sin(lat2_rad))
    
    lat2 = math.degrees(lat2_rad)
    lon2 = math.degrees(lon2_rad)
    
    return lat2, lon2

def meters_to_degrees(meters: float, lat: float) -> Tuple[float, float]:
    """将米转换为纬度和经度方向的度数"""
    # 1度纬度 ≈ 111,000 米
    lat_degrees = meters / 111000.0
    
    # 1度经度 ≈ 111,000 * cos(纬度) 米
    lon_degrees = meters / (111000.0 * math.cos(math.radians(lat)))
    
    return lat_degrees, lon_degrees

def get_obstacle_avoidance_path(start: Tuple[float, float], end: Tuple[float, float], 
                               obstacle_polygon: Polygon, avoidance_distance_m: float, 
                               mode: RouteMode) -> List[Tuple[float, float]]:
    """计算绕过障碍物的路径，确保不穿过整个区域"""
    
    # 检查起点和终点是否在障碍物内
    start_point = Point(start[1], start[0])  # 注意：Shapely使用(经度,纬度)
    end_point = Point(end[1], end[0])
    
    if obstacle_polygon.contains(start_point) or obstacle_polygon.contains(end_point):
        st.warning("起点或终点在障碍物内部，无法规划路径！")
        return [start, end]
    
    # 计算起点到终点的直线
    direct_line = LineString([(start[1], start[0]), (end[1], end[0])])
    
    # 检查直线是否与多边形相交
    if not direct_line.intersects(obstacle_polygon):
        # 不相交，直接返回直线路径
        return [start, end]
    
    # 计算障碍物的平均纬度，用于米到度的转换
    centroid = obstacle_polygon.centroid
    avg_lat = centroid.y
    
    # 将米转换为度
    lat_degrees, lon_degrees = meters_to_degrees(avoidance_distance_m, avg_lat)
    avoidance_distance_degrees = max(lat_degrees, lon_degrees)
    
    # 创建缓冲区域
    buffered_polygon = obstacle_polygon.buffer(avoidance_distance_degrees, resolution=16)
    
    # 如果缓冲后是MultiPolygon，取最大的一个
    if isinstance(buffered_polygon, MultiPolygon):
        # 找到最大的多边形
        max_area = 0
        max_poly = None
        for poly in buffered_polygon.geoms:
            if poly.area > max_area:
                max_area = poly.area
                max_poly = poly
        buffered_polygon = max_poly
    
    # 获取缓冲多边形的边界
    if hasattr(buffered_polygon, 'exterior'):
        boundary = buffered_polygon.exterior
    else:
        # 如果缓冲后是MultiPolygon，取第一个
        boundary = buffered_polygon.geoms[0].exterior
    
    # 获取边界坐标
    boundary_coords = list(boundary.coords)
    
    # 将坐标转换为(纬度, 经度)格式
    boundary_points = [(coord[1], coord[0]) for coord in boundary_coords]
    
    # 找到从起点和终点到缓冲区域边界最近的点
    def find_nearest_point_on_boundary(point: Tuple[float, float], boundary_points: List[Tuple[float, float]]) -> Tuple[int, Tuple[float, float]]:
        """找到边界上离给定点最近的点及其索引"""
        min_dist = float('inf')
        nearest_point = boundary_points[0]
        nearest_index = 0
        
        for i, bp in enumerate(boundary_points):
            dist = calculate_distance(point[0], point[1], bp[0], bp[1])
            if dist < min_dist:
                min_dist = dist
                nearest_point = bp
                nearest_index = i
        
        return nearest_index, nearest_point
    
    start_index, start_nearest = find_nearest_point_on_boundary(start, boundary_points)
    end_index, end_nearest = find_nearest_point_on_boundary(end, boundary_points)
    
    # 根据绕行模式选择方向
    n = len(boundary_points)
    
    if mode == RouteMode.LEFT:
        # 向左绕行（逆时针方向）- 沿边界点向前移动
        if start_index <= end_index:
            waypoints_indices = list(range(start_index, end_index + 1))
        else:
            waypoints_indices = list(range(start_index, n)) + list(range(0, end_index + 1))
        
        waypoints = [boundary_points[i] for i in waypoints_indices]
        
    elif mode == RouteMode.RIGHT:
        # 向右绕行（顺时针方向）- 沿边界点向后移动
        if start_index >= end_index:
            waypoints_indices = list(range(start_index, end_index - 1, -1))
        else:
            waypoints_indices = list(range(start_index, -1, -1)) + list(range(n - 1, end_index - 1, -1))
        
        waypoints = [boundary_points[i] for i in waypoints_indices]
        
    else:  # RouteMode.OPTIMAL
        # 计算两个方向的路径，选择较短的
        # 向左绕行
        if start_index <= end_index:
            left_indices = list(range(start_index, end_index + 1))
        else:
            left_indices = list(range(start_index, n)) + list(range(0, end_index + 1))
        
        left_waypoints = [boundary_points[i] for i in left_indices]
        
        # 向右绕行
        if start_index >= end_index:
            right_indices = list(range(start_index, end_index - 1, -1))
        else:
            right_indices = list(range(start_index, -1, -1)) + list(range(n - 1, end_index - 1, -1))
        
        right_waypoints = [boundary_points[i] for i in right_indices]
        
        # 计算两条路径的长度
        def calculate_path_length(path_points: List[Tuple[float, float]]) -> float:
            if len(path_points) < 2:
                return 0
            total = calculate_distance(start[0], start[1], path_points[0][0], path_points[0][1])
            for i in range(len(path_points) - 1):
                total += calculate_distance(path_points[i][0], path_points[i][1], 
                                          path_points[i+1][0], path_points[i+1][1])
            total += calculate_distance(path_points[-1][0], path_points[-1][1], end[0], end[1])
            return total
        
        left_length = calculate_path_length(left_waypoints)
        right_length = calculate_path_length(right_waypoints)
        
        # 选择较短的路径
        if left_length <= right_length:
            waypoints = left_waypoints
        else:
            waypoints = right_waypoints
    
    # 构建完整路径
    full_path = [start]
    if waypoints and len(waypoints) > 0:
        # 确保路径是连续的
        for i in range(len(waypoints) - 1):
            full_path.append(waypoints[i])
        full_path.append(waypoints[-1])
    full_path.append(end)
    
    return full_path

def calculate_avoidance_path(start: Tuple[float, float], end: Tuple[float, float], 
                            obstacle: Dict, avoidance_distance: float, mode: RouteMode) -> List[Tuple[float, float]]:
    """计算避障路径"""
    # 将障碍物坐标转换为Shapely多边形
    polygon_coords = [(coord[1], coord[0]) for coord in obstacle["coordinates"]]  # 转换为(经度, 纬度)
    obstacle_polygon = Polygon(polygon_coords)
    
    return get_obstacle_avoidance_path(start, end, obstacle_polygon, avoidance_distance, mode)

def update_flight_simulation():
    """更新飞行模拟状态：位置、电量、距离等"""
    if not st.session_state.flight_simulation_active:
        return
    
    if not st.session_state.flight_path or st.session_state.current_waypoint_index >= len(st.session_state.flight_path) - 1:
        # 飞行结束或路径为空
        st.session_state.flight_simulation_active = False
        st.session_state.current_position = st.session_state.flight_path[-1] if st.session_state.flight_path else None
        return
    
    # 获取当前目标航点
    current_target = st.session_state.flight_path[st.session_state.current_waypoint_index + 1]
    current_pos = st.session_state.current_position or st.session_state.flight_path[st.session_state.current_waypoint_index]
    
    # 计算到目标航点的距离和方位
    distance_to_target = calculate_distance(current_pos[0], current_pos[1], 
                                          current_target[0], current_target[1])
    
    # 计算本次更新应移动的距离 (基于速度和心跳间隔)
    move_distance = st.session_state.flight_speed * HEARTBEAT_INTERVAL
    
    if move_distance >= distance_to_target:
        # 到达下一个航点
        st.session_state.current_waypoint_index += 1
        st.session_state.current_position = current_target
        st.session_state.flight_distance_traveled += distance_to_target
    else:
        # 向目标航点移动一段距离
        bearing = calculate_bearing(current_pos[0], current_pos[1], 
                                   current_target[0], current_target[1])
        new_lat, new_lon = calculate_destination_point(current_pos[0], current_pos[1], 
                                                     bearing, move_distance)
        st.session_state.current_position = (new_lat, new_lon)
        st.session_state.flight_distance_traveled += move_distance
    
    # 模拟电量消耗 (每10秒消耗1%)
    now = datetime.now(beijing_tz)
    if st.session_state.flight_start_time:
        elapsed = (now - st.session_state.flight_start_time).total_seconds()
        st.session_state.simulated_battery = max(0, 100 - (elapsed / 10.0))

def generate_heartbeat():
    now = datetime.now(beijing_tz)
    # 如果飞行模拟激活，使用模拟位置
    if st.session_state.flight_simulation_active and st.session_state.current_position:
        lat, lon = st.session_state.current_position
    else:
        # 随机生成位置
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
        "battery": int(st.session_state.simulated_battery) if st.session_state.flight_simulation_active else random.randint(20, 100)
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

def load_obstacles_from_file():
    """从JSON文件加载障碍物数据"""
    if not os.path.exists(OBSTACLE_FILE):
        return []
    try:
        with open(OBSTACLE_FILE, 'r') as f:
            loaded_zones = json.load(f)
        
        # 转换回应用格式：将坐标从列表转换回元组
        obstacle_zones = []
        for zone in loaded_zones:
            # coordinates格式：[[[lat1, lon1], [lat2, lon2], ...]]
            tuple_coords = [tuple(point) for point in zone["coordinates"][0]]
            obstacle_zones.append({
                "coordinates": tuple_coords,
                "height": zone["height"]
            })
        return obstacle_zones
    except Exception as e:
        st.sidebar.error(f"加载障碍物文件失败: {e}")
        return []

def save_obstacles_to_file():
    """将障碍物数据保存到JSON文件"""
    try:
        # 准备可序列化的数据：将坐标从元组转换为列表
        serializable_zones = []
        for zone in st.session_state.obstacle_zones:
            # coordinates格式：[(lat1, lon1), (lat2, lon2), ...]
            list_coords = [[list(point) for point in zone["coordinates"]]]
            serializable_zones.append({
                "coordinates": list_coords,
                "height": zone["height"]
            })
        
        with open(OBSTACLE_FILE, 'w') as f:
            json.dump(serializable_zones, f, indent=2)
    except Exception as e:
        st.sidebar.error(f"保存障碍物文件失败: {e}")

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
            
            # 检查是否已存在相同坐标的障碍物
            existing_coords = [zone["coordinates"] for zone in st.session_state.obstacle_zones]
            if polygon_coords not in existing_coords:
                new_obstacle = {
                    "coordinates": polygon_coords,
                    "height": 50  # 默认高度
                }
                st.session_state.obstacle_zones.append(new_obstacle)
                
    # 保存到文件
    save_obstacles_to_file()

# ===================== 会话状态初始化 =====================
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
    # 从文件加载障碍物数据
    st.session_state.obstacle_zones = load_obstacles_from_file()
if "last_clicked_position" not in st.session_state:
    st.session_state.last_clicked_position = None
if "drag_mode" not in st.session_state:
    st.session_state.drag_mode = None
if "route_mode" not in st.session_state:
    st.session_state.route_mode = RouteMode.OPTIMAL.value
if "calculated_routes" not in st.session_state:
    st.session_state.calculated_routes = {}

# 飞行模拟相关状态
if "flight_simulation_active" not in st.session_state:
    st.session_state.flight_simulation_active = False
if "current_waypoint_index" not in st.session_state:
    st.session_state.current_waypoint_index = 0
if "flight_path" not in st.session_state:
    st.session_state.flight_path = []  # 存储当前飞行的完整航线坐标
if "flight_speed" not in st.session_state:
    st.session_state.flight_speed = 5.0  # 默认飞行速度，单位：米/秒
if "flight_start_time" not in st.session_state:
    st.session_state.flight_start_time = None
if "current_position" not in st.session_state:
    st.session_state.current_position = None
if "flight_distance_traveled" not in st.session_state:
    st.session_state.flight_distance_traveled = 0.0
if "simulated_battery" not in st.session_state:
    st.session_state.simulated_battery = 100  # 初始电量百分比
if "flight_mode" not in st.session_state:
    st.session_state.flight_mode = "IDLE"  # IDLE, FLYING, PAUSED, COMPLETED

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

# 显示飞行模拟状态
if st.session_state.flight_simulation_active:
    st.sidebar.success("✈️ 飞行中")
    st.sidebar.info(f"航点: {st.session_state.current_waypoint_index + 1}/{len(st.session_state.flight_path) if st.session_state.flight_path else 0}")
else:
    st.sidebar.info("✈️ 待机")

st.sidebar.success("A点已设" if st.session_state.point_a else "A点未设")
st.sidebar.success("B点已设" if st.session_state.point_b else "B点未设")
st.sidebar.warning(f"已标记障碍物区域：{len(st.session_state.obstacle_zones)} 个")

# 显示数据文件状态
if os.path.exists(OBSTACLE_FILE):
    file_size = os.path.getsize(OBSTACLE_FILE)
    st.sidebar.info(f"数据文件: {OBSTACLE_FILE} ({file_size} 字节)")
else:
    st.sidebar.info("数据文件: 尚未创建")

# ===================== 页面1：航线规划 =====================
if menu == "航线规划":
    st.title("🗺️ 航线规划 - 无人机避障系统")
    
    # 显示拖拽模式状态
    if st.session_state.drag_mode == 'drag_a':
        st.info("🔴 **拖拽模式激活中**: 点击地图上的任意位置来设置A点的新位置")
    elif st.session_state.drag_mode == 'drag_b':
        st.info("🟢 **拖拽模式激活中**: 点击地图上的任意位置来设置B点的新位置")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        center_lat, center_lon = 32.2422, 118.7490
        if st.session_state.coord_system == "GCJ-02":
            center_lat, center_lon = wgs84_to_gcj02(center_lat, center_lon)

        # 创建地图
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=18,
            min_zoom=10,
            max_zoom=22,
            tiles="cartodb positron",
            prefer_canvas=True,
            zoom_control=True,
            scrollWheelZoom=True,
            dragging=True
        )

        # 卫星图
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='卫星图',
            name='卫星地图',
            max_zoom=18
        ).add_to(m)

        # 2D无限放大图层
        folium.TileLayer(
            tiles='https://cartodb-basemaps-{s}.global.ssl.fastly.net/light_all/{z}/{x}/{y}.png',
            attr='2D平面图',
            name='2D平面地图',
            max_zoom=22
        ).add_to(m)

        folium.LayerControl().add_to(m)

        # 添加A/B点
        if st.session_state.point_a:
            folium.Marker(
                location=st.session_state.point_a, 
                popup="起点A（点击地图重新定位）", 
                tooltip="A点",
                icon=folium.Icon(color="red"),
                draggable=True
            ).add_to(m)
        if st.session_state.point_b:
            folium.Marker(
                location=st.session_state.point_b, 
                popup="终点B（点击地图重新定位）", 
                tooltip="B点",
                icon=folium.Icon(color="green"),
                draggable=True
            ).add_to(m)
        
        # 绘制障碍物
        for i, zone in enumerate(st.session_state.obstacle_zones):
            folium.Polygon(
                locations=zone["coordinates"],
                popup=f"障碍物区域 {i+1}<br>高度: {zone.get('height', 50)} 米",
                color="red",
                fill_color="red",
                fill_opacity=0.3,
                weight=2
            ).add_to(m)
            
            # 绘制障碍物缓冲区（10米）
            try:
                # 创建障碍物多边形
                polygon_coords = [(coord[1], coord[0]) for coord in zone["coordinates"]]
                obstacle_polygon = Polygon(polygon_coords)
                
                # 计算平均纬度用于单位转换
                centroid = obstacle_polygon.centroid
                avg_lat = centroid.y
                
                # 将10米转换为度数
                lat_degrees, lon_degrees = meters_to_degrees(10, avg_lat)
                buffer_distance = max(lat_degrees, lon_degrees)
                
                # 创建缓冲区域
                buffered_polygon = obstacle_polygon.buffer(buffer_distance, resolution=16)
                
                # 获取缓冲区域的边界坐标
                if hasattr(buffered_polygon, 'exterior'):
                    boundary = buffered_polygon.exterior
                elif isinstance(buffered_polygon, MultiPolygon):
                    # 取最大的多边形
                    max_area = 0
                    max_poly = None
                    for poly in buffered_polygon.geoms:
                        if poly.area > max_area:
                            max_area = poly.area
                            max_poly = poly
                    if max_poly and hasattr(max_poly, 'exterior'):
                        boundary = max_poly.exterior
                    else:
                        continue
                else:
                    continue
                
                # 提取边界坐标并转换为(纬度, 经度)格式
                buffer_coords = list(boundary.coords)
                buffer_points = [(coord[1], coord[0]) for coord in buffer_coords]
                
                # 绘制缓冲区
                folium.Polygon(
                    locations=buffer_points,
                    popup=f"障碍物 {i+1} 缓冲区 (10米)",
                    color="orange",
                    fill_color="orange",
                    fill_opacity=0.1,
                    weight=1,
                    dash_array="5, 5"
                ).add_to(m)
                
            except Exception as e:
                st.warning(f"无法绘制障碍物 {i+1} 的缓冲区: {e}")
        
        # 绘制航线
        if st.session_state.point_a and st.session_state.point_b:
            # 绘制直线路径
            folium.PolyLine(
                locations=[st.session_state.point_a, st.session_state.point_b],
                color="gray",
                weight=2,
                dash_array="5, 5",
                popup="直线路径"
            ).add_to(m)
            
            # 绘制计算好的航线
            for mode, path in st.session_state.calculated_routes.items():
                if path and len(path) > 1:
                    color = "blue" if mode == RouteMode.LEFT.value else "green" if mode == RouteMode.RIGHT.value else "red"
                    weight = 4 if mode == st.session_state.route_mode else 2
                    opacity = 1.0 if mode == st.session_state.route_mode else 0.5
                    
                    folium.PolyLine(
                        locations=path,
                        color=color,
                        weight=weight,
                        opacity=opacity,
                        popup=f"{mode}路径"
                    ).add_to(m)
                    
                    # 在关键点添加标记
                    for i, point in enumerate(path):
                        if i > 0 and i < len(path) - 1:  # 中间点
                            folium.CircleMarker(
                                location=point,
                                radius=3,
                                color=color,
                                fill=True,
                                fill_color=color,
                                popup=f"{mode}路径点 {i}"
                            ).add_to(m)

        # 绘制工具
        draw = Draw(
            draw_options={"polyline": False, "polygon": True, "circle": False, "rectangle": False, "marker": False},
            edit_options={"edit": False}
        )
        draw.add_to(m)

        # 渲染地图并获取交互数据
        map_data = st_folium(
            m, 
            width=900, 
            height=600,
            key="map1"
        )
        
        # 处理绘图数据
        if map_data and "all_drawings" in map_data:
            process_draw_data(map_data)
            
        # 处理地图点击事件，用于拖拽模式
        if map_data and map_data.get("last_clicked"):
            clicked_position = map_data["last_clicked"]
            clicked_lat = clicked_position["lat"]
            clicked_lon = clicked_position["lng"]
            
            # 保存最后点击位置
            st.session_state.last_clicked_position = (clicked_lat, clicked_lon)
            
            # 如果在拖拽模式，则更新对应的点
            if st.session_state.drag_mode == 'drag_a':
                st.session_state.point_a = (clicked_lat, clicked_lon)
                st.session_state.drag_mode = None
                st.session_state.calculated_routes = {}  # 清空计算好的航线
                st.success(f"A点已移动到新位置: ({clicked_lat:.6f}, {clicked_lon:.6f})")
                st.rerun()
            elif st.session_state.drag_mode == 'drag_b':
                st.session_state.point_b = (clicked_lat, clicked_lon)
                st.session_state.drag_mode = None
                st.session_state.calculated_routes = {}  # 清空计算好的航线
                st.success(f"B点已移动到新位置: ({clicked_lat:.6f}, {clicked_lon:.6f})")
                st.rerun()

    with col2:
        st.subheader("控制面板")
        
        # 显示当前A/B点坐标
        st.markdown("#### 当前坐标")
        if st.session_state.point_a:
            a_lat, a_lon = st.session_state.point_a
            st.info(f"**A点**: {a_lat:.6f}, {a_lon:.6f}")
        else:
            st.warning("A点未设置")
            
        if st.session_state.point_b:
            b_lat, b_lon = st.session_state.point_b
            st.info(f"**B点**: {b_lat:.6f}, {b_lon:.6f}")
        else:
            st.warning("B点未设置")
        
        st.markdown("---")
        
        # 拖拽功能按钮
        st.markdown("#### 拖拽功能")
        col_drag_a, col_drag_b = st.columns(2)
        with col_drag_a:
            if st.button("拖拽A点", use_container_width=True, 
                         type="primary" if st.session_state.drag_mode == 'drag_a' else "secondary"):
                st.session_state.drag_mode = 'drag_a'
                st.success("拖拽A点模式已激活！请点击地图上的新位置")
                st.rerun()
        with col_drag_b:
            if st.button("拖拽B点", use_container_width=True,
                         type="primary" if st.session_state.drag_mode == 'drag_b' else "secondary"):
                st.session_state.drag_mode = 'drag_b'
                st.success("拖拽B点模式已激活！请点击地图上的新位置")
                st.rerun()
        
        # 取消拖拽模式按钮
        if st.session_state.drag_mode:
            if st.button("取消拖拽模式", use_container_width=True):
                st.session_state.drag_mode = None
                st.warning("拖拽模式已取消")
                st.rerun()
        
        st.markdown("---")
        
        # 航线规划设置
        st.markdown("#### 航线规划设置")
        
        flight_alt = st.slider("设定飞行高度(m)", 10, 100, 30)
        st.info(f"当前设定高度: {flight_alt} m")
        
        # 避障距离设置
        avoidance_distance = st.slider("绕障距离(米)", 5, 50, OBSTACLE_AVOIDANCE_DISTANCE)
        
        # 航线模式选择
        route_mode = st.selectbox(
            "选择航线规划模式",
            [mode.value for mode in RouteMode],
            index=2,  # 默认选择最优路径
            key="route_mode_select"
        )
        st.session_state.route_mode = route_mode
        
        st.info(f"当前模式: {route_mode}")
        
        # 计算航线按钮
        if st.button("计算避障航线", use_container_width=True, type="primary"):
            if not st.session_state.point_a or not st.session_state.point_b:
                st.error("请先设置A点和B点！")
            elif not st.session_state.obstacle_zones:
                st.error("请先绘制障碍物区域！")
            else:
                with st.spinner("计算航线中..."):
                    # 计算三种航线
                    st.session_state.calculated_routes = {}
                    
                    for mode in RouteMode:
                        # 合并所有障碍物为一个大的障碍物区域
                        all_polygons = []
                        for obstacle in st.session_state.obstacle_zones:
                            polygon_coords = [(coord[1], coord[0]) for coord in obstacle["coordinates"]]
                            all_polygons.append(Polygon(polygon_coords))
                        
                        # 合并所有多边形
                        if len(all_polygons) > 1:
                            combined_polygon = unary_union(all_polygons)
                        else:
                            combined_polygon = all_polygons[0]
                        
                        # 计算绕过合并障碍物的路径
                        path = get_obstacle_avoidance_path(
                            st.session_state.point_a,
                            st.session_state.point_b,
                            combined_polygon,
                            avoidance_distance,
                            mode
                        )
                        
                        st.session_state.calculated_routes[mode.value] = path
                    
                    st.success("航线计算完成！")
                    st.rerun()
        
        # 显示航线信息
        if st.session_state.calculated_routes:
            st.markdown("---")
            st.markdown("#### 航线信息")
            
            for mode, path in st.session_state.calculated_routes.items():
                if path and len(path) > 1:
                    # 计算航线总长度
                    total_distance = 0
                    for i in range(len(path) - 1):
                        total_distance += calculate_distance(
                            path[i][0], path[i][1],
                            path[i+1][0], path[i+1][1]
                        )
                    
                    is_selected = mode == st.session_state.route_mode
                    emoji = "✅" if is_selected else "  "
                    
                    st.write(f"{emoji} **{mode}**: {total_distance:.1f}米")
        
        st.markdown("---")
        
        # 手动输入坐标设置A/B点
        st.markdown("#### 手动输入坐标")
        a_lat_input = st.number_input("A点纬度", value=center_lat, format="%.6f", key="a_lat_input")
        a_lon_input = st.number_input("A点经度", value=center_lon, format="%.6f", key="a_lon_input")
        if st.button("手动设置A点", use_container_width=True):
            st.session_state.point_a = (a_lat_input, a_lon_input)
            st.session_state.calculated_routes = {}  # 清空计算好的航线
            st.success(f"A点已设置为: ({a_lat_input:.6f}, {a_lon_input:.6f})")
            st.rerun()
            
        b_lat_input = st.number_input("B点纬度", value=center_lat+0.001, format="%.6f", key="b_lat_input")
        b_lon_input = st.number_input("B点经度", value=center_lon+0.001, format="%.6f", key="b_lon_input")
        if st.button("手动设置B点", use_container_width=True):
            st.session_state.point_b = (b_lat_input, b_lon_input)
            st.session_state.calculated_routes = {}  # 清空计算好的航线
            st.success(f"B点已设置为: ({b_lat_input:.6f}, {b_lon_input:.6f})")
            st.rerun()

        st.markdown("---")
        st.subheader("障碍物区域管理")
        
        # 批量高度设置
        if st.session_state.obstacle_zones:
            new_height = st.slider("批量设置障碍物高度 (米)", 10, 200, 50, key="bulk_height_slider")
            if st.button("应用高度到所有障碍物", use_container_width=True):
                for zone in st.session_state.obstacle_zones:
                    zone["height"] = new_height
                # 保存到文件
                save_obstacles_to_file()
                st.success(f"已将所有障碍物高度设置为 {new_height} 米")
                st.rerun()
        else:
            st.info("绘制障碍物后，可在此设置高度。")
        
        if st.button("保存当前绘制的障碍物", use_container_width=True):
            # 重新处理绘图数据并保存
            if map_data and "all_drawings" in map_data:
                process_draw_data(map_data)
            st.success(f"已保存 {len(st.session_state.obstacle_zones)} 个障碍物区域")
            st.rerun()
        
        if st.button("清除所有障碍物区域", use_container_width=True):
            st.session_state.obstacle_zones = []
            st.session_state.calculated_routes = {}  # 清空计算好的航线
            # 保存空列表到文件
            save_obstacles_to_file()
            st.success("已清除所有障碍物区域")
            st.rerun()
        
        if st.session_state.obstacle_zones:
            with st.expander("查看/编辑障碍物详情"):
                for i, zone in enumerate(st.session_state.obstacle_zones):
                    cols = st.columns([3, 1])
                    with cols[0]:
                        st.write(f"**障碍物 {i+1}** 坐标：")
                        st.caption(f"{zone['coordinates']}")
                    with cols[1]:
                        # 为每个障碍物提供单独的高度编辑器
                        edited_height = st.number_input(
                            f"高度(米)",
                            min_value=1,
                            value=zone["height"],
                            key=f"obs_height_{i}"
                        )
                        if edited_height != zone["height"]:
                            zone["height"] = edited_height
                    st.markdown("---")
                if st.button("应用单独的高度修改", use_container_width=True):
                    # 保存修改后的数据
                    save_obstacles_to_file()
                    st.success("已保存高度修改")
                    st.rerun()
                
            # 导出障碍物数据
            st.markdown("---")
            if st.button("导出障碍物数据", use_container_width=True):
                data_str = json.dumps([
                    {
                        "coordinates": [[list(point) for point in zone["coordinates"]]],
                        "height": zone["height"]
                    }
                    for zone in st.session_state.obstacle_zones
                ], indent=2, ensure_ascii=False)
                st.download_button(
                    label="下载JSON文件",
                    data=data_str,
                    file_name="obstacles_export.json",
                    mime="application/json",
                    use_container_width=True
                )

# ===================== 页面2：飞行监控 =====================
elif menu == "飞行监控":
    st.title("🚁 飞行监控 - 航线动态模拟")
    
    # --- 飞行控制面板 ---
    st.subheader("飞行控制")
    col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4 = st.columns(4)
    
    with col_ctrl1:
        if st.button("✈️ 开始飞行", use_container_width=True, type="primary"):
            # 从航线规划页面获取当前选中的航线
            selected_route = st.session_state.calculated_routes.get(st.session_state.route_mode)
            if selected_route and len(selected_route) > 1:
                st.session_state.flight_path = selected_route
                st.session_state.current_waypoint_index = 0
                st.session_state.current_position = selected_route[0]
                st.session_state.flight_simulation_active = True
                st.session_state.flight_start_time = datetime.now(beijing_tz)
                st.session_state.flight_distance_traveled = 0.0
                st.session_state.simulated_battery = 100
                st.session_state.flight_mode = "FLYING"
                st.success("飞行开始！无人机已从起点起飞。")
            else:
                st.error("无法开始飞行：未找到有效航线。请先在'航线规划'页面计算航线。")
    
    with col_ctrl2:
        if st.button("⏸️ 暂停模拟", use_container_width=True, 
                    disabled=not st.session_state.flight_simulation_active):
            st.session_state.flight_simulation_active = False
            st.session_state.flight_mode = "PAUSED"
            st.warning("飞行模拟已暂停。")
    
    with col_ctrl3:
        if st.button("🔄 重置飞行", use_container_width=True):
            st.session_state.flight_simulation_active = False
            st.session_state.current_waypoint_index = 0
            st.session_state.current_position = None
            st.session_state.flight_distance_traveled = 0.0
            st.session_state.simulated_battery = 100
            st.session_state.flight_start_time = None
            st.session_state.flight_mode = "IDLE"
            st.info("飞行状态已重置。")
    
    with col_ctrl4:
        # 飞行速度设置
        new_speed = st.number_input("速度 (m/s)", min_value=0.5, max_value=20.0, 
                                   value=st.session_state.flight_speed, step=0.5,
                                   key="flight_speed_input")
        st.session_state.flight_speed = new_speed
    
    st.markdown("---")
    
    # --- 监控指标仪表盘 ---
    st.subheader("飞行仪表盘")
    
    # 第一行指标
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        # 当前航点/总航点
        total_wp = len(st.session_state.flight_path) if st.session_state.flight_path else 0
        current_wp = st.session_state.current_waypoint_index + 1 if st.session_state.current_position else 0
        if st.session_state.flight_simulation_active:
            col1.metric("当前航点", f"{current_wp} / {total_wp}", 
                       delta="飞行中" if st.session_state.flight_simulation_active else "就绪")
        else:
            col1.metric("当前航点", f"{current_wp} / {total_wp}", 
                       delta="就绪")
    
    with col2:
        # 飞行速度
        speed = st.session_state.flight_speed if st.session_state.flight_simulation_active else 0.0
        col2.metric("飞行速度", f"{speed:.1f} m/s")
    
    with col3:
        # 已用时间
        elapsed_str = "00:00"
        if st.session_state.flight_start_time and st.session_state.flight_simulation_active:
            elapsed = datetime.now(beijing_tz) - st.session_state.flight_start_time
            total_seconds = int(elapsed.total_seconds())
            minutes, seconds = divmod(total_seconds, 60)
            hours, minutes = divmod(minutes, 60)
            if hours > 0:
                elapsed_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
            else:
                elapsed_str = f"{minutes:02d}:{seconds:02d}"
        col3.metric("已用时间", elapsed_str)
    
    with col4:
        # 剩余距离
        remaining_dist = 0.0
        if st.session_state.flight_path and st.session_state.current_position:
            # 计算到终点的剩余路径总距离
            total_path_dist = 0.0
            current_idx = st.session_state.current_waypoint_index
            # 从当前位置到下一个航点开始计算
            for i in range(current_idx, len(st.session_state.flight_path) - 1):
                if i == current_idx and st.session_state.current_position != st.session_state.flight_path[current_idx]:
                    # 当前在航段中间
                    total_path_dist += calculate_distance(st.session_state.current_position[0], 
                                                         st.session_state.current_position[1],
                                                         st.session_state.flight_path[i+1][0],
                                                         st.session_state.flight_path[i+1][1])
                else:
                    total_path_dist += calculate_distance(st.session_state.flight_path[i][0],
                                                         st.session_state.flight_path[i][1],
                                                         st.session_state.flight_path[i+1][0],
                                                         st.session_state.flight_path[i+1][1])
            remaining_dist = total_path_dist
        col4.metric("剩余距离", f"{remaining_dist:.1f} m")
    
    # 第二行指标
    col5, col6, col7, col8 = st.columns(4)
    
    with col5:
        # 预计到达时间
        eta_str = "--:--"
        if st.session_state.flight_simulation_active and remaining_dist > 0 and st.session_state.flight_speed > 0:
            eta_seconds = remaining_dist / st.session_state.flight_speed
            eta_time = datetime.now(beijing_tz) + timedelta(seconds=eta_seconds)
            eta_str = eta_time.strftime("%H:%M:%S")
        col5.metric("预计到达", eta_str)
    
    with col6:
        # 已飞行距离
        col6.metric("已飞行距离", f"{st.session_state.flight_distance_traveled:.1f} m")
    
    with col7:
        # 剩余航点数
        remaining_wp = max(0, (len(st.session_state.flight_path) if st.session_state.flight_path else 0) - 
                          (st.session_state.current_waypoint_index + 1))
        col7.metric("剩余航点", remaining_wp)
    
    with col8:
        # 模拟电量
        battery = st.session_state.simulated_battery
        battery_color = "normal" if battery > 20 else "inverse"
        if st.session_state.flight_simulation_active:
            col8.metric("模拟电量", f"{battery:.1f}%", delta="- 放电中", delta_color=battery_color)
        else:
            col8.metric("模拟电量", f"{battery:.1f}%", delta="已停止")
    
    st.markdown("---")
    
    # --- 飞行路径可视化地图 ---
    st.subheader("飞行路径与实时位置")
    
    if st.session_state.flight_path and len(st.session_state.flight_path) > 1:
        # 计算地图中心点
        center_lat, center_lon = st.session_state.flight_path[0]
        if st.session_state.current_position:
            center_lat, center_lon = st.session_state.current_position
        if st.session_state.coord_system == "GCJ-02":
            center_lat, center_lon = wgs84_to_gcj02(center_lat, center_lon)
        
        # 创建飞行监控地图
        m2 = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=18,
            tiles="cartodb positron"
        )
        
        # 绘制完整飞行路径
        folium.PolyLine(
            locations=st.session_state.flight_path,
            color="blue",
            weight=3,
            opacity=0.6,
            popup="规划飞行路径"
        ).add_to(m2)
        
        # 绘制航点
        for i, waypoint in enumerate(st.session_state.flight_path):
            color = "green" if i == 0 else "red" if i == len(st.session_state.flight_path)-1 else "gray"
            folium.CircleMarker(
                location=waypoint,
                radius=6 if i in [0, len(st.session_state.flight_path)-1] else 4,
                color=color,
                fill=True,
                fill_color=color,
                popup=f"航点 {i+1}" if i not in [0, len(st.session_state.flight_path)-1] else ("起点" if i==0 else "终点")
            ).add_to(m2)
        
        # 绘制实时位置
        if st.session_state.current_position:
            folium.Marker(
                location=st.session_state.current_position,
                popup=f"无人机实时位置 (航点 {st.session_state.current_waypoint_index+1})",
                tooltip="✈️ 无人机",
                icon=folium.Icon(color="red", icon="plane", prefix="fa")
            ).add_to(m2)
            
            # 绘制从当前位置到下一个航点的连线
            if st.session_state.current_waypoint_index + 1 < len(st.session_state.flight_path):
                next_wp = st.session_state.flight_path[st.session_state.current_waypoint_index + 1]
                folium.PolyLine(
                    locations=[st.session_state.current_position, next_wp],
                    color="green",
                    weight=3,
                    dash_array="10, 10",
                    popup="当前飞行航段"
                ).add_to(m2)
        
        # 渲染地图
        st_folium(m2, width=900, height=500, key="flight_map")
    else:
        st.info("暂无飞行路径。请在'航线规划'页面计算航线并点击'开始飞行'。")
    
    st.markdown("---")
    
    # --- 手动触发飞行状态更新 (用于模拟) ---
    st.subheader("模拟控制")
    col_sim1, col_sim2 = st.columns(2)
    with col_sim1:
        if st.button("🔄 手动更新飞行状态", use_container_width=True):
            if st.session_state.flight_simulation_active:
                update_flight_simulation()
                st.rerun()
            else:
                st.warning("飞行模拟未启动。")
    with col_sim2:
        if st.button("💓 模拟一次心跳", use_container_width=True):
            update_data()  # 原有的心跳更新
            st.rerun()
    
    st.markdown("---")
    
    # --- 保留原有的心跳监控标签页 (可折叠或精简) ---
    with st.expander("📡 原始心跳监控数据", expanded=False):
        col_legacy1, col_legacy2 = st.columns(2)
        with col_legacy1:
            if st.button("重置心跳数据"):
                st.session_state.heartbeat_history = []
                st.session_state.last_heartbeat = None
                st.session_state.drone_status = "未知"
                st.rerun()
        
        status_color = "green" if st.session_state.drone_status == "在线" else "red" if st.session_state.drone_status == "掉线" else "gray"
        col_legacy2.metric("心跳状态", st.session_state.drone_status, delta_color=status_color)
        
        if len(st.session_state.heartbeat_history) >= 2:
            df = pd.DataFrame(st.session_state.heartbeat_history)
            df["timestamp_dt"] = pd.to_datetime(df["timestamp"])
            df["interval_s"] = df["timestamp_dt"].diff().dt.total_seconds()
            fig = px.line(df, x="timestamp", y="interval_s", title="心跳间隔变化曲线")
            fig.add_hline(y=OFFLINE_THRESHOLD, line_dash="dash", line_color="red", annotation_text="掉线阈值")
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("数据不足，无法绘制趋势图")
        
        if st.session_state.heartbeat_history:
            st.dataframe(pd.DataFrame(st.session_state.heartbeat_history), use_container_width=True)

# 初始化数据
if not st.session_state.heartbeat_history:
    for _ in range(10):
        update_data()

# 自动更新飞行状态（如果飞行模拟激活）
if st.session_state.flight_simulation_active:
    # 检查飞行是否结束
    if (st.session_state.flight_path and 
        st.session_state.current_waypoint_index >= len(st.session_state.flight_path) - 1 and
        st.session_state.current_position == st.session_state.flight_path[-1]):
        st.session_state.flight_simulation_active = False
        st.session_state.flight_mode = "COMPLETED"
    else:
        # 自动更新飞行状态
        update_flight_simulation()