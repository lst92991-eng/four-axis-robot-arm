"""Two-link inverse solutions and routes in the measured SolidWorks configuration space."""
import math

import numpy as np


# ---------CAD 工作空间区---------
# 采样表的两个坐标是大臂角和肘角，表值表示该姿态是否在可用连通区。
class Workspace:
    def __init__(self, arm, settings):
        self.arm = arm
        self.units = arm.get('joint_per_drive')
        if self.units is None:  # 历史 JSON 在入口合并方向和比例。
            self.units = (arm['j2_scale']*arm['j2_sign'], arm['j3_sign']/arm['j3_ratio'])
        self.front = settings['neutral_shoulder_deg']
        self.margin = settings['registration_uncertainty_deg']
        self.straight = (settings['transfer_shoulder_deg'] - self.front, 0.)
        self.transfer_speed = settings['transfer_speed_deg_s']
        with np.load(settings['samples'], allow_pickle=False) as data:
            self.angles = data['angles']
            self.free = data['connected']
        if self.free.shape != (len(self.angles), len(self.angles)) or not np.all(np.diff(self.angles) > 0):
            raise ValueError('CAD 角度采样表无效')

    # 把驱动读数换成采样表使用的姿态角。大臂角相对 front，肘角相对大臂，单位为度。
    def angles_of(self, joints):
        a = self.arm
        shoulder = math.degrees(a['shoulder_ref_rad']) + (joints[0] - a['j2_reference_deg'])*self.units[0]
        elbow = math.degrees(a['elbow_ref_rad']) + (joints[1] - a['j3_reference_motor_deg'])*self.units[1]
        return shoulder - self.front, elbow

    # 把规划好的姿态还原为 J2 反馈角与 J3 电机角，供台架下发 MOVE。
    def joints_of(self, pose):
        a = self.arm
        shoulder = pose[0] + self.front - math.degrees(a['shoulder_ref_rad'])
        elbow = pose[1] - math.degrees(a['elbow_ref_rad'])
        # 肘角沿伸直侧展开，避免把穿过完全折叠的短转动当成等价路线。
        return (a['j2_reference_deg'] + shoulder/self.units[0],
                a['j3_reference_motor_deg'] + elbow/self.units[1])

    # 两段臂长和当前肘角决定另一组逆解，它的肘角反号，吸头指向同一 XY。
    def opposite(self, pose):
        upper, lower = self.arm['arm_lengths_mm']
        elbow = math.radians(pose[1])
        turn = math.degrees(2*math.atan2(lower*math.sin(elbow), upper+lower*math.cos(elbow)))
        return math.remainder(pose[0] + turn, 360), -pose[1]

    # 检查两端姿态包围的角度区域，大臂方向还要计入安装对齐余量。
    def free_box(self, a, b):
        bounds = []
        for axis, margin in ((0, self.margin), (1, 0.)):
            low = min(a[axis], b[axis])-margin
            high = max(a[axis], b[axis])+margin
            if low < self.angles[0] or high > self.angles[-1]:
                return False
            first = self.angles.searchsorted(low, side='right')-1
            last = self.angles.searchsorted(high, side='left')
            bounds.append(slice(first, last+1))
        return bool(self.free[tuple(bounds)].all())

    def segment(self, start, end):
        route = []
        while not self.free_box(start, end):
            # 整段通不过时，沿原路线找当前能连续到达的最远点。
            low, high = 0., 1.
            waypoint = start
            for _ in range(16):
                fraction = (low+high)/2
                pose = tuple(x+(y-x)*fraction for x, y in zip(start, end))
                if self.free_box(start, pose):
                    low, waypoint = fraction, pose
                else:
                    high = fraction
            if waypoint == start:
                return None
            route.append(waypoint)
            start = waypoint
        route.append(end)
        return route

    # ---------选择直达或换肘路线区---------
    def route(self, actual, nominal, fixed=False):
        start, goal = self.angles_of(actual), self.angles_of(nominal)
        # 观察位要求固定姿态。普通棋格另算一组逆解，优先检查与当前肘形同侧的解。
        candidates = [goal] if fixed else [goal, self.opposite(goal)]
        candidates.sort(key=lambda p: start[1]*p[1] < 0)
        for end in candidates:
            if not self.free_box(end, end):
                continue
            if start[1]*end[1] >= 0:
                if self.free_box(start, end):
                    return [nominal if end == goal else self.joints_of(end)], False
                path = self.segment(start, end)
                if path is not None:
                    return [self.joints_of(p) for p in path], True
            # 两侧连通时，先到指定的伸直姿态，再从伸直到目标。
            incoming = self.segment(start, self.straight)
            outgoing = self.segment(self.straight, end)
            if incoming is not None and outgoing is not None:
                return [self.joints_of(p) for p in incoming+outgoing], True
        raise ValueError(f'CAD 连通区内找不到从 {actual} 到 {nominal} 的关节路线')

    # 只为目标改变的水平动作保存途经点，每段终点接作下一段起点。
    def plan(self, actual, steps, park):
        routes = {}
        previous = None
        for i,(action, axes, _, j2, j3, _) in enumerate(steps):
            goal = (j2, j3)
            if action != 1 or not axes & 6 or goal == previous:
                continue
            fixed = all(abs(x-y) <= .001 for x,y in zip(goal,park))
            route, transfer = self.route(actual, goal, fixed)
            routes[i] = route, transfer
            actual = route[-1]
            previous = goal
        return routes
