# obstacle_avoidance.py
from controller import Robot
import math



class ObstacleAvoidance:
    def __init__(self, robot, lidar, max_range, max_speed,
                min_valid=0.20,
                 safe_distance=0.50,      # 稍微大一点，离墙 0.5m 内开始强制转向
                 slow_distance=1.20,      # 1.2m 内开始减速
                 emergency_distance=0.20  # 0.4m 以内直接判为“太近了”，先后退
                 ):
        self.robot = robot
        self.lidar = lidar
        self.max_range = max_range
        self.max_speed = max_speed

        self.min_valid = min_valid
        self.safe_distance = safe_distance
        self.slow_distance = slow_distance
        self.emergency_distance = emergency_distance

        # 简单内状态：连续避障的步数（防止一会儿避、一会儿又冲）
        self.avoid_step_counter = 0
        self.AVOID_PERSIST_STEPS = 8  # 进入避障后至少坚持几帧
        # 新增：记住“避障转向方向”（+1 = 左绕，-1 = 右绕）
        self.last_turn_dir = 1

    # ======== LiDAR 扇区处理 ========
    def _get_lidar_sectors(self):
        ranges = self.lidar.getRangeImage()
        if not ranges or len(ranges) == 0:
            return None, None, None

        n = len(ranges)
        third = n // 3
        left_sector   = ranges[0:third]
        center_sector = ranges[third:2 * third]
        right_sector  = ranges[2 * third:n]

        def sector_min(sector):
            finite_vals = [r for r in sector
                           if math.isfinite(r) and r > self.min_valid]
            if len(finite_vals) == 0:
                return self.max_range
            return min(finite_vals)

        left_min   = sector_min(left_sector)
        center_min = sector_min(center_sector)
        right_min  = sector_min(right_sector)

        return left_min, center_min, right_min

    def get_lidar_triplet(self):
        return self._get_lidar_sectors()

    # ======== 主接口：根据 LiDAR 修正轮速 ========
    def apply_avoidance(self, base_left, base_right):
        left_d, center_d, right_d = self._get_lidar_sectors()

        if left_d is None:
            # LiDAR 没数据，不干预
            self.avoid_step_counter = 0
            return base_left, base_right

        d_min = min(left_d, center_d, right_d)

        # 0. 足够安全：完全交给路径规划
        if d_min > self.slow_distance:
            self.avoid_step_counter = 0
            return base_left, base_right

        # === 把路径规划的轮速拆成 平移速度 v 和 角速度 w ===
        # 简单差不考虑轮距：left = v - w, right = v + w
        v = 0.5 * (base_left + base_right)
        w = 0.5 * (base_right - base_left)

        mode = "NONE"

        # 1. 非常近：紧急接管，优先避免撞车
        if d_min <= self.emergency_distance:
            # 离太近了：略微后退 + 原地大转向
            turn_dir = 1 if left_d > right_d else -1  # 左边更空就向左转
            v = -0.1 * self.max_speed          # 小幅后退
            w = 0.6 * self.max_speed * turn_dir
            mode = "EMERGENCY_TURN"

        else:
            # 2. 在 safe_distance ~ slow_distance 之间：
            #    （1）先统一减速
            if self.slow_distance > self.safe_distance:
                ratio = (d_min - self.safe_distance) / (self.slow_distance - self.safe_distance)
                ratio = max(0.2, min(1.0, ratio))  # 限制在 [0.2, 1.0]
            else:
                ratio = 0.5  # 万一参数设置反了，直接减半
            v *= ratio

            #    （2）如果前方比较近 + 左右有明显差别，就往空的一边偏一点
            side_eps = 0.05  # 左右差异阈值，太小就当作一样
            diff_lr = left_d - right_d  # >0 表示左更远

            if center_d < self.safe_distance and abs(diff_lr) > side_eps:
                # 前面也不远了，而且左右明显不一样，开始绕
                turn_dir = 1 if diff_lr > 0 else -1  # 左更远 → 向左绕
                # 根据离 safe_distance 有多近，决定转向强度
                alpha = (self.safe_distance - center_d) / self.safe_distance
                alpha = max(0.0, min(1.0, alpha))  # [0,1]
                w_avoid = turn_dir * alpha * 0.6 * self.max_speed
                w += w_avoid
                mode = "AROUND"
            else:
                # 前方还比较远 或 左右差不多：只减速，不强制拐
                mode = "SLOW_ONLY"

        # === 把 v/w 转回左右轮 ===
        left_wheel  = v - w
        right_wheel = v + w

        # 限幅，防溢出
        left_wheel  = max(-self.max_speed, min(self.max_speed, left_wheel))
        right_wheel = max(-self.max_speed, min(self.max_speed, right_wheel))

        print(
            f"[AVOID] {mode} d_min={d_min:.2f}, "
            f"L={left_d:.2f}, C={center_d:.2f}, R={right_d:.2f}, "
            f"base=({base_left:.2f},{base_right:.2f}) -> "
            f"({left_wheel:.2f},{right_wheel:.2f})"
        )

        return left_wheel, right_wheel
