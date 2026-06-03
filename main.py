import json
import math
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001 
        self.server_ip = '192.168.1.100'

        net = "eJ5ZiZpxr8LN0D0X0col3vA8AElc,192.168.28.1,2182,2183"
        if net != "":
            net = net.split(",")
            self.vehicle_name = net[0]
            self.server_ip = net[1]
            self.udp_port = int(net[2])
            self.udp_send_port = int(net[3])

        print(self.vehicle_name)
        print(self.udp_port)
        print(self.udp_send_port)
        print(self.server_ip)
        self.udp_client = UDPClient(self.server_ip, self.udp_port, self.udp_send_port, self.vehicle_name)

        self.m_v = 0
        self.m_x = 0
        self.m_y = 0
        self.m_yaw = 0
        self.vehpos_initial_index = 0
        self.num_preview = 8
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = 10  # hz
        self.wheel_base = 2.7
        self.prev_speed_cmd = 0.0
        self.prev_steer_cmd = 0.0

    def control_node(self):
        start_time = time.time()
        self.load_route('exp_routes\leftInside.json')
        while True:
            vehicle_data = self.udp_client.get_vehicle_state()
            self.m_x = vehicle_data.x
            self.m_y = vehicle_data.y
            self.m_yaw = vehicle_data.yaw / 180 * math.pi
            self.m_v = 10
            self.update_vehpos_index()
            self.search_target_pos()

            v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
            self.udp_client.send_control_command(v, w)

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def load_route(self, file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            json_track = json.load(file)

        if isinstance(json_track, list):
            self.X_points = [point["x"] for point in json_track]
            self.Y_points = [point["y"] for point in json_track]
        elif isinstance(json_track, dict) and "X" in json_track and "Y" in json_track:
            self.X_points = json_track["X"]
            self.Y_points = json_track["Y"]
        else:
            raise ValueError(
                "Unsupported route format. Expected [{'x': ..., 'y': ...}, ...] "
                "or {'X': [...], 'Y': [...]}."
            )

        if len(self.X_points) != len(self.Y_points) or len(self.X_points) == 0:
            raise ValueError("Route file must contain the same non-zero number of X and Y points.")

        self.X_points = [float(x) for x in self.X_points]
        self.Y_points = [float(y) for y in self.Y_points]

    # def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
    #     ###################################
    #     ##输出控制：速度（v）和转向角（steering_angle）
    #     ##请在此处补全纯跟踪算法的核心计算公式
    #     ##所需参数：
    #     ## m_x, m_y          --车辆位置
    #     ## m_yaw             --车辆航向角
    #     ## target_pos        --目标点
    #     ## self.wheel_base   --轴距
    #     v, steering_angle=self.m_v, 0

    #     ###################################
    #     w = v * math.tan(steering_angle) / self.wheel_base
    #     return v, w

    # def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
    #     ###################################
    #     ## 输出控制：速度（v）和转向角（steering_angle）
        
    #     tx, ty = target_pos

    #     # 1. 计算车辆当前位置到预瞄目标点的距离 (L_d)
    #     dx = tx - m_x
    #     dy = ty - m_y
    #     ld = math.hypot(dx, dy)

    #     # 2. 计算目标点相对于车身坐标系的航向误差角 (alpha)
    #     # math.atan2(dy, dx) 是目标点在全局坐标系下的绝对角度
    #     alpha = math.atan2(dy, dx) - m_yaw
        
    #     # 将 alpha 归一化到 [-pi, pi] 之间，防止多圈角度引起的逻辑翻转
    #     while alpha > math.pi: 
    #         alpha -= 2.0 * math.pi
    #     while alpha < -math.pi: 
    #         alpha += 2.0 * math.pi

    #     # 3. 纯跟踪核心公式：计算前轮转向角 (steering_angle)
    #     if ld > 0.001:  # 防止除 0 异常
    #         steering_angle = math.atan2(2.0 * self.wheel_base * math.sin(alpha), ld)
    #     else:
    #         steering_angle = 0.0

    #     # 4. 个性化动态速度控制：基于曲率自适应的纵向速度控制
    #     # 核心思想：直道全速前进，弯道根据方向盘打角程度平滑减速，防止侧滑漂移
    #     max_v = 10.0  # 直线行驶时的最大期望速度
    #     min_v = 3.0   # 急弯行驶时的最低安全保底速度
        
    #     # 将转向角映射为 0~1 的弯道惩罚系数 (假设常规最大转向角约为 pi/4 即 45度)
    #     turn_penalty = min(abs(steering_angle) / (math.pi / 4.0), 1.0)
        
    #     # 采用二次平滑衰减函数：入弯初期轻微减速，弯道越急减速越狠
    #     v = max_v - (max_v - min_v) * (turn_penalty ** 2)

    #     ###################################
        
    #     # 运动学转换：阿克曼转向角转化为中心偏航角速度
    #     w = v * math.tan(steering_angle) / self.wheel_base
    #     return v, w   

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def get_route_turn_ratio(self, start_index, preview_count=18):
        route_size = len(self.X_points)
        if route_size < 4:
            return 0.0

        heading_changes = []
        for i in range(min(preview_count, route_size - 2)):
            i0 = (start_index + i) % route_size
            i1 = (start_index + i + 1) % route_size
            i2 = (start_index + i + 2) % route_size

            dx1 = self.X_points[i1] - self.X_points[i0]
            dy1 = self.Y_points[i1] - self.Y_points[i0]
            dx2 = self.X_points[i2] - self.X_points[i1]
            dy2 = self.Y_points[i2] - self.Y_points[i1]
            if math.hypot(dx1, dy1) < 1e-6 or math.hypot(dx2, dy2) < 1e-6:
                continue

            h1 = math.atan2(dy1, dx1)
            h2 = math.atan2(dy2, dx2)
            heading_changes.append(abs(self.normalize_angle(h2 - h1)))

        if not heading_changes:
            return 0.0
        return min(max(heading_changes) / math.radians(8.0), 1.0)

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        """
        纯跟踪横向控制 + 按转弯强度自适应的纵向速度控制。
        返回 Unity 控制接口需要的线速度 v 和角速度 w。
        """
        target_x, target_y = target_pos
        dx = target_x - m_x
        dy = target_y - m_y
        ld = math.hypot(dx, dy)

        if ld < 1e-6:
            return 0.0, 0.0

        alpha = self.normalize_angle(math.atan2(dy, dx) - m_yaw)

        # 弯前使用较短预瞄，减少切弯；直道保持稍长预瞄来抑制画龙。
        route_turn = self.get_route_turn_ratio(self.vehpos_initial_index)
        lookahead_floor = 5.0 - 1.8 * route_turn
        lookahead_distance = max(ld, lookahead_floor)
        steering_angle = math.atan2(
            2.0 * self.wheel_base * math.sin(alpha),
            lookahead_distance
        )

        max_steer = math.radians(32.0)
        steering_angle = max(min(steering_angle, max_steer), -max_steer)

        lateral_error = abs(ld * math.sin(alpha))
        steer_ratio = min(abs(steering_angle) / max_steer, 1.0)
        error_ratio = min(lateral_error / 3.0, 1.0)
        turn_ratio = max(steer_ratio, route_turn, 0.8 * error_ratio)

        max_speed = 9.0
        min_speed = 3.2
        desired_speed = max_speed - (max_speed - min_speed) * (turn_ratio ** 1.4) 

        # 目标点已经落到车身后方时，优先低速纠偏，防止横穿进草坪。
        if abs(alpha) > math.radians(85.0):
            desired_speed = min(desired_speed, min_speed)

        if self.prev_speed_cmd <= 0.0:
            self.prev_speed_cmd = desired_speed

        dt = 1.0 / self.control_rate
        max_steer_step = math.radians(90.0) * dt
        steer_delta = steering_angle - self.prev_steer_cmd
        steer_delta = max(min(steer_delta, max_steer_step), -max_steer_step)
        steering_angle = self.prev_steer_cmd + steer_delta

        max_accel = 2.0
        max_decel = 5.0
        speed_delta = desired_speed - self.prev_speed_cmd
        speed_delta = max(min(speed_delta, max_accel * dt), -max_decel * dt)
        v = self.prev_speed_cmd + speed_delta

        self.prev_steer_cmd = steering_angle
        self.prev_speed_cmd = v

        w = v * math.tan(steering_angle) / self.wheel_base
        return v, w


    def search_vehicle_initial_index(self):
        min_distance = float('inf')
        nearest_index = 0

        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        self.vehpos_initial_index = nearest_index

    
    def find_nearest_point_index(self, target_x, target_y):
        min_distance = float('inf')
        nearest_index = -1
    
        for i in range(len(self.X_points)):
            this_point_x = self.X_points[i]
            this_point_y = self.Y_points[i]

            distance = math.sqrt((target_x - this_point_x) ** 2 + (target_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = i

        return nearest_index
    
    def update_vehpos_index(self):
        min_distance = float('inf')
        nearest_index = 0
        for i in range(40):
            find_index = (self.vehpos_initial_index + i) % len(self.X_points)
            this_point_x = self.X_points[find_index]
            this_point_y = self.Y_points[find_index]

            distance = math.sqrt((self.m_x - this_point_x) ** 2 + (self.m_y - this_point_y) ** 2)

            if distance < min_distance:
                min_distance = distance
                nearest_index = find_index
        if min_distance > 25:
            self.search_vehicle_initial_index()
        else:
            self.vehpos_initial_index = nearest_index

    def search_target_pos(self):
        route_size = len(self.X_points)
        if route_size == 0:
            return

        # 弯道前缩短预瞄，目标点沿路径弧长推进，避免提前吸到弯后的路径点。
        route_turn = self.get_route_turn_ratio(self.vehpos_initial_index)
        preview_distance = self.num_preview - 3.2 * route_turn
        preview_distance = max(4.0, min(self.num_preview, preview_distance))

        target_pos_index = self.vehpos_initial_index
        accumulated_distance = 0.0
        for i in range(route_size - 1):
            current_index = (self.vehpos_initial_index + i) % route_size
            next_index = (current_index + 1) % route_size
            segment_length = math.hypot(
                self.X_points[next_index] - self.X_points[current_index],
                self.Y_points[next_index] - self.Y_points[current_index]
            )
            accumulated_distance += segment_length
            target_pos_index = next_index
            if accumulated_distance >= preview_distance:
                break

        self.targetPos_Info[0] = self.X_points[target_pos_index]
        self.targetPos_Info[1] = self.Y_points[target_pos_index]

if __name__ == '__main__':
    control = Control()
    control.udp_client.start()
    control.control_node()
