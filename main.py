import json
import math
import os
import time
from my_udp import UDPClient


class Control:
    def __init__(self):

        self.vehicle_name = '1'
        self.udp_port = 9000
        self.udp_send_port = 9001 
        self.server_ip = '192.168.1.100'

        net = "eJ5ZiZpxr8LN0D0X0col3vA8AElc,192.168.28.1,7986,7987"
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
        self.route_file = 'exp_routes/birth13_to_big_2000.json'
        self.num_preview = 14
        self.targetPos_Info = [0.0, 0.0]
        self.Y_points = []
        self.X_points = []
        self.control_rate = 10  # hz
        self.wheel_base = 2.7
        self.prev_speed_cmd = 0.0
        self.prev_steer_cmd = 0.0
        self.prev_lateral_error = 0.0
        self.prev_log_time = None
        self.prev_log_speed = 0.0
        self.run_start_time = None
        self.log_file = None
        self.blocked_since = None
        self.overtake_until = 0.0
        self.overtake_offset = 0.0
        self.start_pose_printed = False
        self.max_competition_speed = 18.5
        self.safety_stop_distance = 5.0
        self.follow_time_gap = 1.2

    def control_node(self):
        start_time = time.time()
        self.run_start_time = start_time
        self.init_run_log()
        self.load_route(self.route_file)
        while True:
            vehicle_data = self.udp_client.get_vehicle_state()
            if vehicle_data.name == "":
                time.sleep(1.0 / self.control_rate)
                start_time = time.time()
                continue

            self.m_x = vehicle_data.x
            self.m_y = vehicle_data.y
            self.m_yaw = vehicle_data.yaw / 180 * math.pi
            self.m_v = min(vehicle_data.speed, self.max_competition_speed)
            if not self.start_pose_printed and vehicle_data.name != "":
                print(
                    "START_POSE",
                    "name=", vehicle_data.name,
                    "x=%.3f" % self.m_x,
                    "y=%.3f" % self.m_y,
                    "yaw_deg=%.3f" % vehicle_data.yaw,
                    "speed=%.3f" % vehicle_data.speed
                )
                self.start_pose_printed = True
            self.update_vehpos_index()
            self.search_target_pos()

            v, w = self.calc_pure_pursuit(self.m_x, self.m_y, self.m_yaw, self.targetPos_Info)
            v, w = self.obstacle_avoidance(self.m_x, self.m_y, self.m_yaw, v, w)
            v = min(v, self.max_competition_speed)
            self.udp_client.send_control_command(v, w)
            self.write_run_log(vehicle_data, v, w)

            elapsed_time = time.time() - start_time
            sleep_time = max((1.0 / self.control_rate) - elapsed_time, 0.0)
            time.sleep(sleep_time)
            start_time = time.time()

    def init_run_log(self):
        os.makedirs("logs", exist_ok=True)
        file_name = time.strftime("run_%Y%m%d_%H%M%S.csv")
        log_path = os.path.join("logs", file_name)
        self.log_file = open(log_path, "w", encoding="utf-8", buffering=1)
        self.log_file.write(
            "time,x,y,yaw_deg,speed,accel,cmd_v,cmd_w,route_index,target_x,target_y\n"
        )
        print("RUN_LOG", log_path)

    def write_run_log(self, vehicle_data, cmd_v, cmd_w):
        if self.log_file is None:
            return

        now = time.time()
        if self.prev_log_time is None:
            accel = 0.0
        else:
            dt = max(now - self.prev_log_time, 1e-6)
            accel = (vehicle_data.speed - self.prev_log_speed) / dt

        self.prev_log_time = now
        self.prev_log_speed = vehicle_data.speed
        elapsed = now - self.run_start_time if self.run_start_time is not None else 0.0
        self.log_file.write(
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%d,%.3f,%.3f\n" % (
                elapsed,
                self.m_x,
                self.m_y,
                vehicle_data.yaw,
                vehicle_data.speed,
                accel,
                cmd_v,
                cmd_w,
                self.vehpos_initial_index,
                self.targetPos_Info[0],
                self.targetPos_Info[1],
            )
        )

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

    def get_path_heading(self, index):
        route_size = len(self.X_points)
        if route_size < 2:
            return self.m_yaw

        next_index = (index + 1) % route_size
        return math.atan2(
            self.Y_points[next_index] - self.Y_points[index],
            self.X_points[next_index] - self.X_points[index]
        )

    def get_signed_lateral_error(self, m_x, m_y, index):
        if len(self.X_points) < 2:
            return 0.0

        path_heading = self.get_path_heading(index)
        dx = m_x - self.X_points[index]
        dy = m_y - self.Y_points[index]
        return -math.sin(path_heading) * dx + math.cos(path_heading) * dy

    def project_to_route(self, m_x, m_y):
        route_size = len(self.X_points)
        if route_size < 2:
            return self.vehpos_initial_index, m_x, m_y, self.m_yaw, 0.0

        best_distance = float('inf')
        best_result = (self.vehpos_initial_index, self.X_points[0], self.Y_points[0], self.m_yaw, 0.0)
        for offset in range(0, 70):
            index = (self.vehpos_initial_index + offset) % route_size
            next_index = (index + 1) % route_size
            x0 = self.X_points[index]
            y0 = self.Y_points[index]
            x1 = self.X_points[next_index]
            y1 = self.Y_points[next_index]
            sx = x1 - x0
            sy = y1 - y0
            segment_length_sq = sx * sx + sy * sy
            if segment_length_sq < 1e-9:
                continue

            t = ((m_x - x0) * sx + (m_y - y0) * sy) / segment_length_sq
            t = max(0.0, min(1.0, t))
            proj_x = x0 + t * sx
            proj_y = y0 + t * sy
            dx = m_x - proj_x
            dy = m_y - proj_y
            distance = math.hypot(dx, dy)
            if distance < best_distance:
                heading = math.atan2(sy, sx)
                segment_length = math.sqrt(segment_length_sq)
                ux = sx / segment_length
                uy = sy / segment_length
                signed_error = ux * dy - uy * dx
                best_distance = distance
                best_result = (index, proj_x, proj_y, heading, signed_error)

        return best_result

    def get_priority_value(self, name):
        text = str(name)
        return sum((i + 1) * ord(ch) for i, ch in enumerate(text))

    def obstacle_avoidance(self, m_x, m_y, m_yaw, v, w):
        """
        多车安全监督器：覆盖基础前向避障、跟车、路口冲突和对向会车让行。
        返回修正后的速度和角速度，保证速度不超过比赛限制。
        """
        neighbor_vehicle_data = self.udp_client.get_neighbor_vehicle_state()
        if not neighbor_vehicle_data:
            self.blocked_since = None
            return min(v, self.max_competition_speed), w

        my_priority = self.get_priority_value(self.vehicle_name)
        speed_limit = self.max_competition_speed
        command_speed = min(v, speed_limit)
        command_w = w
        should_yield = False
        emergency_stop = False
        nearest_obstacle = float('inf')

        for other_vehicle in neighbor_vehicle_data:
            dx = other_vehicle.x - m_x
            dy = other_vehicle.y - m_y
            distance = math.hypot(dx, dy)
            if distance < 1e-6 or distance > 32.0:
                continue
            nearest_obstacle = min(nearest_obstacle, distance)

            target_yaw = math.atan2(dy, dx)
            relative_angle = self.normalize_angle(target_yaw - m_yaw)
            front_distance = max(8.0, self.safety_stop_distance + command_speed * self.follow_time_gap)
            route_turn = self.get_route_turn_ratio(self.vehpos_initial_index, preview_count=8)

            # 极近距离不区分方向，先保命避免碰撞违规。
            if distance <= 3.5:
                emergency_stop = True
                continue

            # 基础避障和跟车：前方扇形内根据距离连续降速，近距离强制停车。
            if abs(relative_angle) <= math.radians(40.0) and distance <= front_distance:
                is_stopped_front = other_vehicle.speed < 0.5 and distance > self.safety_stop_distance + 1.5
                if is_stopped_front and route_turn < 0.08:
                    self.overtake_until = time.time() + 5.5
                    # 负偏移优先向路径右侧绕行；本路线起步车道右侧留有道路空间。
                    self.overtake_offset = -2.6
                    command_speed = min(command_speed, 6.0)
                elif distance <= self.safety_stop_distance:
                    emergency_stop = True
                else:
                    # 慢车跟随：按前车速度和安全时距连续降速，不急刹。
                    safe_speed = max(
                        2.0,
                        min(other_vehicle.speed + 1.0, (distance - self.safety_stop_distance) / self.follow_time_gap)
                    )
                    command_speed = min(command_speed, safe_speed)

            # 路口/车道竞争：近距离侧前方车辆按固定优先级让行，避免双方抢占。
            is_cross_conflict = (
                distance <= 14.0
                and abs(relative_angle) <= math.radians(100.0)
                and abs(self.normalize_angle(other_vehicle.yaw - m_yaw)) > math.radians(35.0)
            )
            if is_cross_conflict and my_priority > self.get_priority_value(other_vehicle.name):
                should_yield = True
                command_speed = min(command_speed, 3.0)

            # 对向会车或狭窄道路死锁：低优先级车让行，但避免超过 10 秒完全停车。
            is_head_on = (
                distance <= 14.0
                and abs(relative_angle) <= math.radians(30.0)
                and abs(self.normalize_angle(other_vehicle.yaw - m_yaw)) > math.radians(135.0)
            )
            if is_head_on and my_priority > self.get_priority_value(other_vehicle.name):
                should_yield = True
                command_speed = min(command_speed, 2.0)

        if emergency_stop:
            self.blocked_since = self.blocked_since or time.time()
            if time.time() - self.blocked_since < 8.0 or nearest_obstacle <= self.safety_stop_distance:
                return 0.0, 0.0
            return 0.8, 0.0

        if should_yield:
            self.blocked_since = self.blocked_since or time.time()
            if time.time() - self.blocked_since > 8.0:
                command_speed = max(command_speed, 1.0)
        else:
            self.blocked_since = None

        return max(0.0, min(command_speed, speed_limit)), command_w

    def calc_pure_pursuit(self, m_x, m_y, m_yaw, target_pos):
        """
        Stanley/PID 横向控制 + 曲率限速。
        保留函数名以兼容主循环，实际控制不再依赖单点纯跟踪。
        返回 Unity 控制接口需要的线速度 v 和角速度 w。
        """
        route_index, _, _, path_heading, lateral_error = self.project_to_route(m_x, m_y)
        self.vehpos_initial_index = route_index
        route_turn = self.get_route_turn_ratio(route_index, preview_count=8)
        speed_turn = self.get_route_turn_ratio(route_index, preview_count=26)
        heading_error = self.normalize_angle(path_heading - m_yaw)
        speed_for_control = max(abs(self.m_v), 5.0)
        dt = 1.0 / self.control_rate
        self.prev_lateral_error = lateral_error
        active_overtake = time.time() < self.overtake_until
        target_lateral_offset = self.overtake_offset if active_overtake else 0.0
        lateral_error_to_target = lateral_error - target_lateral_offset

        is_straight = route_turn < 0.08
        if is_straight and not active_overtake:
            if abs(heading_error) < math.radians(1.2) and abs(lateral_error_to_target) < 0.35:
                steering_angle = 0.0
            else:
                steering_angle = (
                    0.32 * heading_error
                    - math.atan2(0.16 * lateral_error_to_target, speed_for_control)
                )
        else:
            stanley_gain = 0.28 + 0.42 * route_turn
            steering_angle = (
                0.62 * heading_error
                - math.atan2(stanley_gain * lateral_error_to_target, speed_for_control)
            )

        # 出生点前几米的路径是专门设计的直行并线段，限制初始修正避免直接冲草坪。
        if route_index < 8:
            steering_angle = max(min(0.45 * steering_angle, math.radians(8.0)), -math.radians(8.0))

        max_steer = math.radians(32.0)
        steering_angle = max(min(steering_angle, max_steer), -max_steer)

        steer_ratio = min(abs(steering_angle) / max_steer, 1.0)
        error_ratio = min(abs(lateral_error_to_target) / 2.8, 1.0)
        heading_ratio = min(abs(heading_error) / math.radians(35.0), 1.0)
        turn_ratio = max(0.75 * steer_ratio, speed_turn, 0.9 * error_ratio, 0.7 * heading_ratio)

        max_speed = self.max_competition_speed
        min_speed = 4.5
        desired_speed = max_speed - (max_speed - min_speed) * (turn_ratio ** 1.05)

        if route_index < 8:
            desired_speed = min(desired_speed, 5.0)
        if active_overtake:
            desired_speed = min(desired_speed, 8.0)
        if abs(lateral_error_to_target) > 1.2:
            desired_speed = min(desired_speed, 9.0)
        if abs(heading_error) > math.radians(70.0):
            desired_speed = min(desired_speed, min_speed)

        if self.prev_speed_cmd <= 0.0:
            self.prev_speed_cmd = min(desired_speed, max(self.m_v, min_speed))

        max_steer_step = math.radians(28.0 + 45.0 * route_turn) * dt
        steer_delta = steering_angle - self.prev_steer_cmd
        steer_delta = max(min(steer_delta, max_steer_step), -max_steer_step)
        steering_angle = self.prev_steer_cmd + steer_delta

        max_accel = 3.5
        max_decel = 10.0
        speed_delta = desired_speed - self.prev_speed_cmd
        speed_delta = max(min(speed_delta, max_accel * dt), -max_decel * dt)
        v = self.prev_speed_cmd + speed_delta

        self.prev_steer_cmd = steering_angle
        self.prev_speed_cmd = v

        w = v * math.tan(steering_angle) / self.wheel_base
        max_w = 0.12 if is_straight and not active_overtake else 0.28 + 0.85 * route_turn
        w = max(min(w, max_w), -max_w)
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

        # 高速直道看远一点抑制画龙；弯道缩短预瞄，避免提前吸到弯后的路径点。
        route_turn = self.get_route_turn_ratio(self.vehpos_initial_index)
        preview_distance = self.num_preview + 0.15 * self.m_v - 4.0 * route_turn
        preview_distance = max(6.5, min(18.0, preview_distance))

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
