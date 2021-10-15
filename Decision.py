from pyroutelib3 import Router  # Import the router
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import os
import csv
import math
import copy
import pymap3d as pm
import cubic_spline_planner
from quintic_polynomials_planner import QuinticPolynomial
from multiprocessing import Process, Value
from datetime import datetime
# CAN_library-------------------------
from canlib import canlib, kvadblib
from canlib.canlib import ChannelData

dbc_llh = kvadblib.Dbc(filename='02_GPS_INS_Parser_1206.dbc')
dbc_end = kvadblib.Dbc(filename='06_Path_1027.dbc')
db_lidar = kvadblib.Dbc(filename='05_LiDAR_1005.dbc')
dbc_vel = kvadblib.Dbc(filename='01_Avante_dbc_0710.dbc')

#---------------------------Vehicle----------------------------------
Vehicle_info_2 = dbc_vel.get_message_by_name('Vehicle_Info_2') # -> For Speed
Vehicle_info_2_sig = Vehicle_info_2.bind()
Vehicle_info_4 = dbc_vel.get_message_by_name('Vehicle_info_4') # -> For turn left right
Vehicle_info_4_sig = Vehicle_info_4.bind()
#---------------------------Vehicle----------------------------------

#---------------------------GPS----------------------------------
RTK_GPS_Latitude = dbc_llh.get_message_by_name('RTK_Latitude')
RTK_GPS_Latitude_sig = RTK_GPS_Latitude.bind()

RTK_GPS_Longitude = dbc_llh.get_message_by_name('RTK_Longitude')
RTK_GPS_Longitude_sig = RTK_GPS_Longitude.bind()
#---------------------------GPS----------------------------------

#---------------------------Path----------------------------------
Wave_Path_Next_Lat = dbc_end.get_message_by_name('Wave_Path_Next_Lat')
Wave_Path_Next_Lat_sig = Wave_Path_Next_Lat.bind()

Wave_Path_Next_Long = dbc_end.get_message_by_name('Wave_Path_Next_Long')
Wave_Path_Next_Long_sig = Wave_Path_Next_Long.bind()

Wave_Path_Next_Alt_Yaw = dbc_end.get_message_by_name('Wave_Path_Next_Alt_Yaw')
Wave_Path_Next_Alt_Yaw_sig = Wave_Path_Next_Alt_Yaw.bind()

Wave_StopLine_Lat = dbc_end.get_message_by_name('Wave_StopLine_Lat')
Wave_StopLine_Lat_sig = Wave_StopLine_Lat.bind()

Wave_StopLine_Long = dbc_end.get_message_by_name('Wave_StopLine_Long')
Wave_StopLine_Long_sig = Wave_StopLine_Long.bind()
#---------------------------Path----------------------------------


# Parameter of the LPP=========================================================
TARGET_SPEED = 60.0 / 3.6  # target speed [m/s]
MAX_ACCEL = 10.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 10.0  # maximum curvature [1/m]
LENGTH = 4.37  # [m]
WIDTH = 1.53  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.1  # [m]
WHEEL_WIDTH = 0.05  # [m]
TREAD = 0.7  # [m]
WB = 2.5  # [m]
MAX_SPEED = 100.0 / 3.6  # maximum speed [m/s]
MAX_LEFT_WIDTH = -15
MAX_RIGHT_WIDTH = 15
D_ROAD_W = 1.0  # road width sampling length [m]
K_J = 0.1  # cost weights
K_T = 0.1  # cost weights
K_D = 1.0  # cost weights
K_LAT = 1.0  # cost weights
K_LON = 1.0  # cost weights
MAX_T = 6.0  # max prediction time [m]
MIN_T = 4.0  # min prediction time [m]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 0.1  # sampling number of target speed
DT = 0.3  # time tick [s] [if value higher, simulation program getting faster and vice versa]
ROBOT_RADIUS = 3  # robot radius [m] [if value higher, get enable angle of obstacle avoidance length]
SHORTE = 4
LARGE = 15

Out_of_speed = False
Out_of_accel = False
Out_of_curvature = False
Warning_obstacle = False
show_animation = True
# Parameter of the LPP=========================================================

# Setting CAN Channels=========================================================
# CAN Channel Finding
def find_channel(ch_num):
    return_channel_num = 99
    num = canlib.getNumberOfChannels() # 연결되어 있는 모든 채널들을 가져온다

    for channel in range(0, num):
        chdata = canlib.ChannelData(channel)

        print("%d, %s, %s, %s" % (channel, chdata.channel_name, chdata.card_upc_no, chdata.card_serial_no))
        ean = canlib.getChannelData_EAN(channel)

        if '11111-1' in ean :   # ex) 11111-1은 Kvaser Device Guide에서 연결된 CAN Device EAN '00-00000-11111-1'의 11111-1을 확인 후 기입한다.
            return_channel_num = channel + ch_num
            break

    return return_channel_num

# CAN FD Channel Finding
def find_channel_fd(ch_num):
    return_channel_num = 99
    num = canlib.getNumberOfChannels()

    for channel in range(0, num):
        chdata = canlib.ChannelData(channel)

        print("%d, %s, %s, %s" % (channel, chdata.channel_name, chdata.card_upc_no, chdata.card_serial_no))
        ean = canlib.getChannelData_EAN(channel)
        # ean = ChannelData(channel).card_upc_no

        if '11111-1' in ean : # ex) 11111-1은 Kvaser Device Guide에서 연결된 CAN Device EAN '00-00000-11111-1'의 11111-1을 확인 후 기입한다.
            return_channel_num = channel + ch_num
            break

    return return_channel_num

# CAN Channel을 Setup한다.
def setUpChannel(channel, openFlags=canlib.canOPEN_ACCEPT_VIRTUAL, bitrate=canlib.canBITRATE_500K, bitrateFlags=canlib.canDRIVER_NORMAL):
    ch = canlib.openChannel(channel, openFlags) # 채널을 오픈한다
    print("Using channel: %s, EAN: %s" % (ChannelData(channel).device_name,
                                          ChannelData(channel).card_upc_no)
                                              )
    ch.setBusOutputControl(bitrateFlags)
    ch.setBusParams(bitrate)
    ch.busOn()
    return ch

def tearDownChannel(ch):
    ch.busOff()
    ch.close()
# Setting CAN Channels=========================================================

# FRENET FRAME ALGORITHM=======================================================
class FrenetPath:
    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt

def calc_frenet_paths(c_speed, lateral_position, lateral_speed, acceleration, current_course_position):
    frenet_paths = []

    # meter of -road to road one by one
    # calculate 하는 범위를 지정해 주는 곳 map_enable_lane
    for di in np.arange(MAX_LEFT_WIDTH, MAX_RIGHT_WIDTH, D_ROAD_W):
        # Lateral motion planning
        for ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()
            lat_qp = QuinticPolynomial(lateral_position, lateral_speed, acceleration, di, 0.0, 0.0, ti)

            fp.t = [t for t in np.arange(0.0, ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(current_course_position, c_speed, 0.0, tv, 0.0, ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)
    return frenet_paths

def calc_global_paths(fplist, index):
    for fp in fplist:
        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = index.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = index.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.hypot(dx, dy))

        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist

def check_collision(fp, obstacle):
    # print("obstacle ",obstacle)
    if len(obstacle) > 0:
        for i in range(len(obstacle[:, 0])):
            # print("obstacle ", len(obstacle[:, 0]))
            d = [((ix - obstacle[i, 0]) ** 2 + (iy - obstacle[i, 1]) ** 2)
                 for (ix, iy) in zip(fp.x, fp.y)]
            collision = any([di <= ROBOT_RADIUS ** 2 for di in d])
            if collision:
                return False
        return True
    else:
        return True

def check_paths(fplist, obstacle):
    global Out_of_speed
    global Out_of_accel
    global Out_of_curvature
    global Warning_obstacle

    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            Out_of_speed = True
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            Out_of_accel = True
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            Out_of_curvature = True
            continue
        elif not check_collision(fplist[i], obstacle):
            Warning_obstacle = True
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

# Local Path Planning Algorithm
def frenet_optimal_planning(index, current_course_position, c_speed, lateral_position, lateral_speed, acceleration,
                            obstacle):
    fplist = calc_frenet_paths(c_speed, lateral_position, lateral_speed, acceleration, current_course_position)
    fplist = calc_global_paths(fplist, index)
    fplist = check_paths(fplist, obstacle)
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp
    return best_path
# FRENET FRAME ALGORITHM=======================================================

# start get_course-------------------
def get_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

# Plotting Vehicle-------------------------
def plot_car(x, y, yaw=0.0, WIDTH=1.53, LENGTH=4.37, steer=0.0, cabcolor="-r", truckcolor="-k",
             BACKTOWHEEL=1.0, WHEEL_LEN=0.15, WHEEL_WIDTH=0.05, TREAD=0.7, WB=2.5):

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
                          -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "ok", label="My Vehicle", markersize=5)

    return outline

# Receive all information by can------------------------------------------------
def My_Position(current_lat, current_lon, ALL_STOP):
    channel_num = find_channel(1) # Find channel with CAN
    channel = setUpChannel(channel_num) # SET Channel
    while True:
        if ALL_STOP.value == 1:
            break
        try:
            frame = channel.read() # read channel
            if frame.id == 0x401: # 이 채널에는 차량의 Latitude가 있음
                RTK_GPS_Latitude_sig = RTK_GPS_Latitude.bind(frame)
                S_lat_int = int(RTK_GPS_Latitude_sig.RTK_Lat_Int.phys)
                S_lat_double = float(RTK_GPS_Latitude_sig.RTK_Lat_Dec.phys)
                current_lat.value = S_lat_int + (S_lat_double / 100000000)

            elif frame.id == 0x402: # 이 채널에는 차량의 Longitude가 있음
                RTK_GPS_Longitude_sig = RTK_GPS_Longitude.bind(frame)
                S_lon_int = int(RTK_GPS_Longitude_sig.RTK_Long_Int.phys)
                S_lon_double = float(RTK_GPS_Longitude_sig.RTK_Long_Dec.phys)
                current_lon.value = S_lon_int + (S_lon_double / 100000000)

        except (canlib.canNoMsg) as ex:
            pass
    tearDownChannel(channel)
    print("CAR INFORMATION Parshing stop")

def gpp2lpp(velocity, current_lat, current_lon, light, ALL_STOP):
    # Initialize---------------------------------------------------------------
    global Out_of_speed
    global Out_of_accel
    global Out_of_curvature
    global Warning_obstacle
    global MAX_LEFT_WIDTH
    global MAX_RIGHT_WIDTH
    global show_animation

    matplotlib.use('TkAgg')
    location = './'
    Directory = os.listdir(location) # 경로에 있는 모든 파일들을 불러온다
    channel_num = find_channel(2) # For Sending information to Control System
    Channel = setUpChannel(channel_num)

    Coordination = []
    stop_line = []
    HD_LIST = []
    
    # HD Map 정보가 담겨져 있는 csv 파일을 가져온다
    for name in Directory:
        file_name = name.split('.')
        if len(file_name) > 1:
            if file_name[1] == 'csv':
                HD_LIST.append(name)
    
    
    for name in HD_LIST:
        with open(name, 'r', encoding='UTF8') as f:
            # Necessary----------------------------------------------
            if name == "A2_LINK.csv": # 이 파일은 HD Map의 모든 노드 데이터들이 있는 파일이다. (필수 데이터)
                all_data = csv.reader(f)
                coord = []
                for line in all_data:
                    if not line == []:
                        temp = [float(x) for x in line]
                        coord.append(temp)
                Coordination = np.array(coord)
            # Necessary----------------------------------------------
            # OPTIONS-------------------------------------------------
            elif name == "A2_STOP.csv": # 이 파일은 HD Map의 Stop Line 데이터들이 있는 파일이다. (옵션 데이터)
                all_data = csv.reader(f)
                tt = []
                for line in all_data:
                    if not line == []:
                        temp = [float(x) for x in line]
                        tt.append(temp)
                stop_line = np.array(tt)
            # OPTIONS-------------------------------------------------

    router = Router("car", "A2_LINK_OUT.osm")  # HD Map이 있는 모든 데이터들을 가져온다. Readme.md 설명 참조
    # Initialize---------------------------------------------------------------

    # 차량의 위도 및 경도를 받았을 때 while문을 나온다
    while True:
        if current_lat.value > 0 and current_lon.value > 0:
            lat = current_lat.value
            lon = current_lon.value
            break

    # Start -----------------------------------------------------------------
    start = router.findNode(lat, lon) # 나의 좌표
    end = router.findNode(37.1111111, 126.11111111) # 목표 지점 좌표

    status, route = router.doRoute(start, end)  # Find the route - a list of OSM nodes (Global Path Planning)

    if status == 'success':
        routeLatLons = list(map(router.nodeLatLon, route))  # Get actual route coordinates
        routeLatLons = np.array(routeLatLons)

        # Essential (Change list)
        result_path = []
        for latlon in routeLatLons:
            result_path.append([float(latlon[0]), float(latlon[1])])
        result_path = np.array(result_path)

        # plotting Coordination------------------------------------------ option
        plt.title("Global Path Planning")
        plt.plot(Coordination[:, 1], Coordination[:, 0], '*b', label="HD Map")
        plt.plot(result_path[:, 1], result_path[:, 0], '--r', linewidth=3, label='Generated Path Trajectory')
        plt.plot(result_path[0, 1], result_path[0, 0], 'Xm', markersize=10, label="Start Point")
        plt.plot(result_path[-1, 1], result_path[-1, 0], 'Xg', markersize=10, label="End Point")
        plt.plot(stop_line[:, 1], stop_line[:, 0], 'ok', markersize=5, label="Stop Lines")
        plt.legend()
        plt.show()
        # plotting Coordination------------------------------------------ option

        center = sum(Coordination) / len(Coordination) # Calculate ENU Center For Convert other LLH Data to ENU

        # Create all lane------------------------- option
        ENU_all = []
        for llh in Coordination:
            e, n, u = pm.geodetic2enu(llh[0], llh[1], llh[2], center[0], center[1], center[2])
            ENU_all.append([e, n])
        ENU_all = np.array(ENU_all)
        # ------------------------------------------ option

        # Create enu pqosition of the GPP result
        temp_result = []
        for llh in result_path:
            e, n, u = pm.geodetic2enu(llh[0], llh[1], center[2], center[0], center[1], center[2])
            temp_result.append([e, n])
        result_path = np.array(temp_result)

        # Create Stop Line by me
        traffic_all = []
        for llh in stop_line:
            e, n, u = pm.geodetic2enu(llh[0], llh[1], llh[2], center[0], center[1], center[2])
            traffic_all.append([e, n])
        traffic_all = np.array(traffic_all)

        # This is Local Path Planning----------------------------------
        dx, dy, dyaw, dk, s = get_course(result_path[:, 0], result_path[:, 1]) # X, Y Yaw, Curvature, Index

        # Initial Parameters ------------------------------------
        # For LPP
        c_d = 0.0  # current lateral position [m]
        c_d_d = 0.0  # current lateral speed [m/s]
        c_d_dd = 0.0  # current lateral acceleration [m/s]
        # s0 = 0.0  # current course position
        area = 50.0  # animation area length [m]
        # For LPP
        # For Comparing
        heading = 0
        time = 0.0
        toggle = False
        temp_light = 0
        # For Comparing
        # ----------------------------------------------
        lists_dict = {}
        for index, lists in enumerate(result_path):
            lists_dict[index] = lists

        start_time = datetime.now() # For Compare Times
        obstacle = [[0, 0], [1, 1]] #example Obstacle

        # LOCAL PATH PLANNING Start!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        while True:
            if ALL_STOP.value == 1: # IF all stop toggle on, All Codes Stop
                break
            # Change my Coordinate to ENU
            comp_x, comp_y, comp_z = pm.geodetic2enu(current_lat.value, current_lon.value, center[2], center[0], center[1], center[2])

            # Compare my Coordination--------------------------
            temp_dict = {}
            for key, value in lists_dict.items():
                point_position = np.hypot(value[0] - comp_x, value[1] - comp_y)
                temp_dict[key] = point_position
            POINT = min(temp_dict.keys(), key=lambda k: temp_dict[k])
            s0 = s.s[POINT]

            # For Car's left right light----------------------
            # 차선 바꾸고 나서 가고 있는데 장애물 회피해야 됨, 그러면 이거
            if Warning_obstacle == True:
                temp_light = 0

            # 깜빡이 처리============================================================
            end_time = datetime.now()
            SEC = (end_time - start_time).seconds  # Second
            if SEC > 3: # 만약 3초 동안 깜빡이 없으면
                toggle = False
                if temp_light == 0: # 이전에 켜진 깜박이가 없거나, 옆으로 갔다가 다시 돌아올 때
                    # 정밀지도 상에서 헤딩에 따라 피하고 난 다음 위치 선정 -> 헤딩이 일정 구간 안에 있으면 왼쪽 or 오른쪽
                    if heading > 150 and heading < 250: # 왼쪽으로 피함
                        MAX_LEFT_WIDTH = -LARGE
                        MAX_RIGHT_WIDTH = 1
                    elif heading > 300 or heading < 50: # 오른쪽으로 피함
                        MAX_LEFT_WIDTH = -1
                        MAX_RIGHT_WIDTH = LARGE
                    elif temp_light < 0:  # 오른쪽으로만 계속 주행할 수 있도록 설정
                        MAX_LEFT_WIDTH = -LARGE * abs(temp_light)
                        MAX_RIGHT_WIDTH = -SHORTE * abs(temp_light)
                    elif temp_light > 0:  # 왼쪽으로만 계속 주행할 수 있도록 설정
                        MAX_LEFT_WIDTH = SHORTE * abs(temp_light)
                        MAX_RIGHT_WIDTH = LARGE * abs(temp_light)

            if light.value == -1: # 오른쪽 신호를 받았을 때
                start_time = datetime.now()
                if toggle == False: # 한번만 실행되게 만드는것
                    idx = np.abs(np.array(dx) - path.x[-1]).argmin() # 맨 앞의 LPP Position 값이 전체 구역에서 몇번째 index 가지고 있는지 추출 (X)
                    idy = np.abs(np.array(dy) - path.y[-1]).argmin() # 맨 앞의 LPP Position 값이 전체 구역에서 몇번째 index 가지고 있는지 추출 (Y)
                    obstacle.append([dx[idx], dy[idy]]) # 자리 이동할 수 있도록 하나의 장애물을 위에 추가
                    temp_light += light.value # 만약 오른쪽 신호를 받고 그 쪽으로 갔는데, 또 오른쪽 신호를 받으면 피할 ROI를 더 증가하도록 하기 위한 것
                    toggle = True # 한번 신호 받으면 그 다음 이 if 문으로 안 들어 오게 하는 toggle
                if temp_light == 0:
                    MAX_LEFT_WIDTH = -SHORTE
                    MAX_RIGHT_WIDTH = SHORTE
                elif temp_light < 0:
                    MAX_LEFT_WIDTH = LARGE * temp_light  # 오른쪽으로 피함
                    MAX_RIGHT_WIDTH = SHORTE * temp_light
                else:
                    MAX_LEFT_WIDTH = SHORTE * temp_light  # 왼쪽으로 피함
                    MAX_RIGHT_WIDTH = LARGE * temp_light

            if light.value == 1: # 왼쪽 신호를 받았을 때
                start_time = datetime.now()
                if toggle == False:
                    idx = np.abs(np.array(dx) - path.x[-1]).argmin() # 맨 앞의 LPP Position 값이 전체 구역에서 몇번째 index 가지고 있는지 추출 (X)
                    idy = np.abs(np.array(dy) - path.y[-1]).argmin() # 맨 앞의 LPP Position 값이 전체 구역에서 몇번째 index 가지고 있는지 추출 (Y)
                    obstacle.append([dx[idx], dy[idy]]) # 자리 이동할 수 있도록 하나의 장애물을 위에 추가
                    temp_light += light.value
                    toggle = True # 한번 신호 받으면 그 다음 이 if 문으로 안 들어 오게 하는 toggle
                if temp_light == 0:
                    MAX_LEFT_WIDTH = -SHORTE
                    MAX_RIGHT_WIDTH = SHORTE
                elif temp_light < 0:
                    MAX_LEFT_WIDTH = LARGE * temp_light  # 오른쪽으로 피함
                    MAX_RIGHT_WIDTH = SHORTE * temp_light
                else:
                    MAX_LEFT_WIDTH = SHORTE * temp_light  # 왼쪽으로 피함
                    MAX_RIGHT_WIDTH = LARGE * temp_light
            # 깜빡이 처리============================================================

            # For Car's left right light----------------------
            obstacle = np.array(obstacle)  # Relate Coordination

            path = frenet_optimal_planning(s, s0, velocity.value, c_d, c_d_d, c_d_dd, obstacle)
            c_d = path.d[1]
            c_d_d = path.d_d[1]
            c_d_dd = path.d_dd[1]

            time = time + DT

            heading = math.atan2(path.x[1] - path.x[0], path.y[1] - path.y[0]) * 180 / math.pi + 180 # 현재 값과 다음 값의 방향을 비교해서 Heading값 추출하기
            # update my coordination ---------------------------------

            # stop line 찾는 법은 간단하게 코사인 유사도 거리 방법 및 유클라디안 거리 검출 사용함====================
            INDEXS = {}
            point = np.array([path.x[0], path.y[0]])
            last_point = np.array([path.x[int(len(path.x) / 2)], path.y[int(len(path.y) / 2)]])
            comp_heading = math.atan2(last_point[0] - point[0], last_point[1] - point[1]) * 180 / math.pi + 180

            for i in range(len(traffic_all)):
                comp = np.array(traffic_all[i])
                point_position = np.array(comp - point)
                # Heading ----------->
                heading_traffic = math.atan2(point_position[0], point_position[1]) * 180 / math.pi + 180
                point_pos = np.sqrt((point_position[0] ** 2) + (point_position[1] ** 2))  # L2 norm vector -> 유클라디안 거리 검출임
                if abs(abs(heading_traffic) - abs(comp_heading)) < 10: # 대충 Cosine 유사도 거리 검출임
                    INDEXS[i] = point_pos

            if len(INDEXS) > 0:
                POINT = min(INDEXS.keys(), key=lambda k: INDEXS[k])
            else:
                POINT = 0
            stop_lat, stop_lon, stop_h = pm.enu2geodetic(traffic_all[POINT][0], traffic_all[POINT][1], center[2],
                                                         center[0], center[1], center[2])
            # stop line 찾는 법은 간단하게 코사인 유사도 거리 방법 및 유클라디안 거리 검출 사용함====================
            
            # Find next Coordination based on Lookahead=====================================================================
            lookahead = velocity.value * 0.2 + 4.5
            num = 0
            for i in range(len(path.x)):
                distance = np.hypot(path.x[i] - path.x[0], path.y[i] - path.y[0])
                dists = distance - lookahead
                if dists > 0:
                    num = i
                    break
            # ENU to LLH
            next_lat, next_lon, next_alt = pm.enu2geodetic(path.x[num], path.y[num], center[2], center[0], center[1], center[2])
            # Find next Coordination based on Lookahead=====================================================================
            
            # tx버퍼 클리어 구문
            # 실제 실험할 때 키는 구문 (제어 프로세스에 송신)==============================
            canlib.IOControl(Channel).flush_tx_buffer()
            Wave_Path_Next_Lat_sig.Wave_Path_Next_Lat.phys = round(float(next_lat), 8)
            Channel.write(Wave_Path_Next_Lat_sig._frame)
            Wave_Path_Next_Long_sig.Wave_Path_Next_Long.phys = round(float(next_lon), 7)
            Channel.write(Wave_Path_Next_Long_sig._frame)
            Wave_Path_Next_Alt_Yaw_sig.Wave_Path_Next_Alt.phys = round(float(next_alt), 2)
            Channel.write(Wave_Path_Next_Alt_Yaw_sig._frame)
            Wave_StopLine_Lat_sig.Wave_StopLine_Lat.phys = float(stop_lat)
            Channel.write(Wave_StopLine_Lat_sig._frame)
            Wave_StopLine_Long_sig.Wave_StopLine_Long.phys = float(stop_lon)
            Channel.write(Wave_StopLine_Long_sig._frame)
            # 실제 실험할 때 키는 구문 (제어 프로세스에 송신)==============================

            # Warning Alert=================================
            if Out_of_speed == True:
                print("[Speed] -> Out of the Max")
                Out_of_speed = False

            if Out_of_accel == True:
                print("[Accel] -> Out of the Max")
                Out_of_accel = False

            if Out_of_curvature == True:
                print("[Curvature] -> Out of the Max")
                Out_of_curvature = False

            if Warning_obstacle == True:
                print("[WARNING] -> Obstacle")
                Warning_obstacle = False
            # Warning Alert=================================

            # 자동차 제동 거리 공식
            break_distance = round(0.0005 * math.pow(velocity.value * 3.6, 2) + 0.2 * (velocity * 3.6), 3)
            comp_end = np.hypot(path.x[1] - dx[-1], path.y[1] - dy[-1])
            # 자동차 제동 거리와 비교해서 다달으면 모든 코드 break
            if comp_end <= break_distance:
                print("Goal!")
                ALL_STOP.value = 1
                break

            if show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

                # line of the world
                plt.plot(ENU_all[:, 0], ENU_all[:, 1], '*b')
                # line of the GPP
                plt.plot(dx, dy, '--k', linewidth=3)
                if len(obstacle) > 0:
                    plt.plot(obstacle[:, 0], obstacle[:, 1], "or", markersize=6)
                # line of the LPP
                plt.plot(path.x[1:], path.y[1:], "-og", linewidth=2)
                plot_car(path.x[1], path.y[1], radian)
                plt.plot(traffic_all[:, 0], traffic_all[:, 1], '*y')
                plt.plot(traffic_all[POINT][0], traffic_all[POINT][1], 'Xm', markersize=10)
                # ROI
                plt.xlim(path.x[1] - area, path.x[1] + area)
                plt.ylim(path.y[1] - area, path.y[1] + area)

                text = "Time: " + str(round(time, 2)) + " / Velocity: " + str(round(velocity * 3.6, 2)) + "km/h"
                plt.title(text)
                plt.grid(True)
                plt.pause(0.0001)
        print("Finish")
    else:
        print("Position not found. Tray again...")
    print("GPP=LPP FINISH")

def light_parshing(velocity, light, ALL_STOP):
    # Initialize---------------------------------------------------------------
    # Can channel ------------------------
    channel_num = find_channel_fd(0) # For Receive Vehicle Information with CAN FD
    channel = setUpChannel(channel_num) #set channel CAN FD
    # Can channel ------------------------

    while True:
        if ALL_STOP.value == 1:
            break
        try:
            frame = channel.read() # 채널에서 들어오는 데이터를 읽어들인다

            if frame.id == 0x54: # 이 Data Field ID에는 차량의 깜빡이 데이터가 출력된다
                Vehicle_info_4_sig = Vehicle_info_4.bind(frame) # frame에 있는 데이터들을 불러 들인다
                left = Vehicle_info_4_sig.Turn_Sig_Left_Lamp.phys # Left Light에 대한 데이터 값을 읽어들인다.
                right = Vehicle_info_4_sig.Turn_Sig_Right_Lamp.phys # Right Light에 대한 데이터 값을 읽어들인다.

                if left == 1: # 만약 Left Light가 들어왔다면
                    light.value = 1 # Process Value 1을 올려줌
                elif right == 1: # 만약 Right Light가 들어왔다면
                    light.value = -1 # Process Value -1을 올려줌
                else:
                    light.value = 0 # 모두 다 안 들어올 시 0으로 반환

            elif frame.id == 0x52: # 이 Data Field ID에는 차량의 속도 데이터가 출력된다
                Vehicle_info_2_sig = Vehicle_info_2.bind(frame) # frame에 있는 데이터들을 불러 들인다
                velocity.value = Vehicle_info_2_sig.Vehicle_Speed.phys #차량 데이터를 읽어드린다.

        except (canlib.canNoMsg) as ex:
            pass
    tearDownChannel(channel) # Open한 채널을 닫는다
    print("Light Parshing Clear")

if __name__ == "__main__":
    ALL_STOP = Value('i', 0) # For Stop All Process Codes
    velocity = Value('d', 0.0) # Velocity Data
    current_lat = Value('d', 0.0) # Vehicle's Latitude Data
    current_lon = Value('d', 0.0) # Vehicle's Longitude Data
    light = Value('i', 0) # Vehicle's Turn Left or Right Data

    # Vehicle's Information of the Latitude and Longitude Data
    V = Process(target=My_Position, args=(current_lat, current_lon, ALL_STOP))
    V.start()

    # Global Path Planning and Local Path Planning Algorithms
    GPP_LPP = Process(target=gpp2lpp, args=(velocity, current_lat, current_lon, light, ALL_STOP))
    GPP_LPP.start()

    # Vehicle's Information of the Velocity and Left or Right light
    left_right_light = Process(target=light_parshing, args=(velocity, light, ALL_STOP))
    left_right_light.start()
