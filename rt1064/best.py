import numpy as np
from collections import deque, defaultdict
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass
from typing import Set, FrozenSet, List, Tuple
import functools
import heapq
import time
import tracemalloc
'''m中只有153不行'''
''' fail'''
m153 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 3, 1, 0, 5, 1, 0, 0, 1, 0, 0, 0, 1],
    [1, 3, 1, 2, 2, 0, 0, 0, 1, 0, 2, 0, 1],
    [1, 3, 1, 0, 0, 1, 0, 2, 1, 0, 0, 0, 1],
    [1, 3, 1, 0, 2, 1, 0, 0, 1, 0, 2, 1, 1],
    [1, 3, 1, 0, 0, 1, 0, 2, 1, 0, 0, 1, 1],
    [1, 3, 1, 0, 2, 1, 0, 0, 1, 0, 2, 1, 1],
    [1, 3, 3, 0, 0, 1, 0, 2, 0, 0, 0, 1, 1],
    [1, 3, 3, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
'''step=19 easy'''
m150 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
    [1, 0, 0, 2, 0, 0, 5, 3, 3, 1],
    [1, 0, 2, 0, 0, 0, 0, 1, 0, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 0, 1],
    [1, 1, 1, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
r7 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1],
    [1, 1, 0, 1, 0, 5, 1, 1, 0, 2, 2, 0, 1],
    [1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 2, 0, 0, 1, 1, 1, 0, 0, 0, 1],
    [1, 1, 1, 0, 1, 1, 1, 1, 1, 2, 1, 1, 1],
    [1, 0, 2, 0, 0, 1, 1, 1, 0, 3, 3, 1, 1],
    [1, 0, 2, 0, 2, 0, 2, 0, 3, 3, 3, 1, 1],
    [1, 0, 0, 0, 0, 1, 1, 1, 3, 3, 3, 1, 1],
    [1, 0, 2, 2, 0, 1, 1, 1, 3, 3, 3, 1, 1],
    [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
r20 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 1, 5, 0, 1, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0, 0, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 2, 0, 3, 3, 1],
    [1, 0, 1, 0, 2, 0, 0, 2, 0, 0, 1, 0, 0, 1, 0, 0, 1, 3, 1],
    [1, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 0, 3, 3, 1],
    [1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 3, 1],
    [1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 2, 0, 3, 3, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 0, 2, 1, 1, 1, 1, 0, 0, 1, 3, 1],
    [1, 0, 1, 2, 0, 0, 0, 2, 0, 0, 2, 0, 0, 1, 0, 2, 3, 3, 1],
    [1, 0, 0, 0, 0, 2, 0, 1, 0, 2, 0, 2, 0, 1, 0, 0, 1, 3, 1],
    [1, 1, 1, 1, 0, 2, 1, 1, 1, 0, 0, 0, 0, 1, 0, 2, 3, 3, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 2, 2, 0, 1, 1, 1, 3, 3, 3, 3, 1],
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
# 0=空地 1=墙体 2=箱子 3=目的地 4=炸弹 5=小车
'''
========================================
     寻路耗时      : 5.0895 秒
     峰值内存占用  : 15.1243 MB
     step = 103
     累计展开节点数: 5,350 个
     节点处理速度  : 1,051 节点/秒
========================================'''
r1 = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,0,0,0,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,2,0,0,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,0,0,2,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,0,0,2,0,2,0,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,0,1,0,1,1,0,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,1,0,1,1,0,1,1,1,1,1,0,0,3,3,1],
    [1,0,2,0,0,2,0,0,0,0,0,0,0,0,0,0,3,3,1],
    [1,1,1,1,1,0,1,1,1,0,1,5,1,1,0,0,3,3,1],
    [1,1,1,1,1,0,0,0,0,0,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
]
'''  '''
r2 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 1, 5, 0, 1, 1, 1],
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0, 0, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 2, 0, 3, 3, 1],
    [1, 0, 1, 0, 2, 0, 0, 2, 0, 0, 1, 0, 0, 1, 0, 0, 1, 3, 1],
    [1, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 0, 3, 3, 1],
    [1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 3, 1]
]
r3 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # 第0行
    [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1],  # 第1行
    [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 1, 5, 0, 1, 1, 1],  # 第2行
    [1, 0, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0, 0, 1, 1, 1],  # 第3行
    [1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 2, 0, 3, 3, 1],  # 第4行
    [1, 0, 1, 0, 2, 0, 0, 2, 0, 0, 1, 0, 0, 1, 0, 0, 1, 3, 1],  # 第5行
    [1, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 1, 2, 0, 3, 3, 1],  # 第6行
    [1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 3, 1]   # 第7行
]
'''========================================
 寻路耗时      : 7.4454 秒
 峰值内存占用  : 13.5933 MB
 step = 36
 累计展开节点数: 5,608 个
 节点处理速度  : 753 节点/秒
========================================'''
system_map3 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 3, 1],
    [1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 1],
    [1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 0, 3, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1],
    [1, 5, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1],
    [1, 0, 4, 1, 0, 0, 0, 0, 0, 0, 0, 4, 1, 0, 0, 1],
    [1, 0, 0, 2, 0, 1, 0, 0, 0, 0, 0, 2, 4, 0, 0, 1],
    [1, 0, 2, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1],
    [1, 0, 0, 0, 0, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
l328 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
    [1, 0, 0, 0, 0, 0, 0, 2, 5, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 0, 3, 1, 2, 2, 2, 0, 2, 0, 1],
    [1, 0, 3, 0, 3, 0, 1, 0, 0, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 0, 3, 1, 0, 2, 2, 0, 2, 0, 1],
    [1, 0, 3, 0, 3, 0, 1, 0, 0, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 0, 3, 1, 2, 0, 2, 0, 2, 0, 1],
    [1, 0, 3, 0, 3, 0, 1, 0, 0, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 1, 3, 0, 2, 0, 2, 0, 2, 0, 1],
    [1, 0, 3, 0, 3, 0, 1, 0, 0, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 0, 3, 1, 1, 0, 2, 0, 2, 0, 1],
    [1, 0, 3, 0, 3, 0, 0, 1, 0, 2, 0, 2, 0, 1],
    [1, 3, 0, 3, 0, 3, 0, 1, 0, 0, 0, 0, 0, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
l864 = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 1, 0, 0, 1, 1, 0, 0, 2, 5, 2, 0, 1, 1],
    [1, 1, 0, 3, 0, 0, 0, 1, 1, 1, 3, 0, 1, 1],
    [1, 1, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
    [1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1],
    [1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 1, 0, 0, 1],
    [1, 1, 1, 0, 0, 0, 1, 1, 0, 3, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 0, 2, 0, 0, 1, 1, 0, 1, 1],
    [1, 1, 0, 3, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1],
    [1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1],
    [1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
# --- 常量定义 ---
HEIGHT = len(m150)
WIDTH = len(m150[0])
MAP_SIZE = WIDTH * HEIGHT
DIRS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右
N = 2
# 【终极参数调优区】
VIRTUAL_WALL_COST = 20  # 启发式穿墙代价：引导算法发现抄近道的可能
BOMB_PENALTY = int(0.8 * VIRTUAL_WALL_COST)

flag=1 #0不分类 1分类
box_ids={}   #dict, {一维坐标: 编号}
goal_ids={}  #dict, {一维坐标: 编号}

def to_1d(x: int, y: int) -> int:
    return y * WIDTH + x
def to_2d(idx: int) -> Tuple[int, int]:
    return idx % WIDTH, idx // WIDTH

# --- 1. 状态定义 ---
@dataclass(frozen=True)
class State:
    car_pos: int
    boxes: FrozenSet[Tuple[int, int]]
    bombs: FrozenSet[int]
    walls: FrozenSet[int]
    goals: FrozenSet[Tuple[int, int]]


@dataclass
class MacroAction:
    item_start_idx: int
    item_end_idx: int
    car_stand_pos: int
    is_bomb_exploded: bool
    is_consumed: bool


# --- 2. 核心求解器类 ---
class SokobanSolver:
    def __init__(self, raw_map_2d: List[List[int]],flag: int = 0, box_ids: dict = None, goal_ids: dict = None):
        """
                :param flag: 0=无视分类，1=启用分类
                :param box_ids: dict, {一维坐标: 编号}
                :param goal_ids: dict, {一维坐标: 编号}
        """
        # 预计算路径时，只需要知道目标点的物理位置
        self.initial_goal_positions: FrozenSet[int] = frozenset()
        self.boundary_walls: FrozenSet[int] = frozenset()
        self.initial_state: State
        self.safe_mask: set[int] = set()
        self.explosion_masks: dict = {}

        self.parse_map(raw_map_2d, flag, box_ids, goal_ids)
        self.precalc_safe_mask()
        self.precalc_explosion_masks()

        self.visited_states: dict = {}
        self.h_cache = {}
        self.solution_actions: List[MacroAction] = []
        self.total_explored_nodes = 0
        self.current_weight = 0

    def parse_map(self, raw_map, flag, box_ids, goal_ids):
        walls, bombs = set(), set()
        boxes_with_id = set()
        goals_with_id = set()
        goal_positions = set()
        temp_boundary_walls = set()
        car_pos = -1

        # 如果缺省字典，初始化为空字典防止报错
        box_ids = box_ids or {}
        goal_ids = goal_ids or {}

        for y in range(HEIGHT):
            for x in range(WIDTH):
                val = raw_map[y][x]
                idx = to_1d(x, y)
                if val == 1:
                    walls.add(idx)
                    if x == 0 or x == WIDTH - 1 or y == 0 or y == HEIGHT - 1:
                        temp_boundary_walls.add(idx)
                elif val == 2:
                    # 【大一统处理】：flag=0强行设为0；flag=1取输入字典，找不到默认0
                    b_id = box_ids.get(idx, 0) if flag == 1 else 0
                    boxes_with_id.add((idx, b_id))
                elif val == 3:
                    g_id = goal_ids.get(idx, 0) if flag == 1 else 0
                    goals_with_id.add((idx, g_id))
                    goal_positions.add(idx)
                elif val == 4:
                    bombs.add(idx)
                elif val == 5:
                    car_pos = idx
        self.initial_goal_positions = frozenset(goal_positions)
        self.boundary_walls = frozenset(temp_boundary_walls)
        self.initial_state = State(
            car_pos=car_pos,
            boxes=frozenset(boxes_with_id),
            bombs=frozenset(bombs),
            walls=frozenset(walls),
            goals=frozenset(goals_with_id)
        )

    def precalc_safe_mask(self):
        """逆向拉拽搜索 (Reverse Pull BFS)"""
        # 如果图中有炸弹，墙体可能随时消失，为保证安全禁用此剪枝
        if self.initial_state.bombs:
            self.safe_mask = set(range(MAP_SIZE))
            return

        self.safe_mask = set(self.initial_goal_positions)
        queue = deque(self.initial_goal_positions)
        visited = set(self.initial_goal_positions)
        while queue:
            curr = queue.popleft()
            cx, cy = to_2d(curr)
            for dx, dy in DIRS:
                # 箱子的前一个位置 (px, py)，小车的拉拽站位 (ppx, ppy)
                px, py = cx - dx, cy - dy
                ppx, ppy = px - dx, py - dy
                # 1. 严格越界检测
                if not (0 <= px < WIDTH and 0 <= py < HEIGHT): continue
                if not (0 <= ppx < WIDTH and 0 <= ppy < HEIGHT): continue

                p_idx = to_1d(px, py)
                pp_idx = to_1d(ppx, ppy)
                # 2. 只有当箱子和小车的位置都不是墙壁时，动作合法
                if (p_idx not in self.initial_state.walls) and (pp_idx not in self.initial_state.walls):
                    if p_idx not in visited:
                        visited.add(p_idx)
                        self.safe_mask.add(p_idx)
                        queue.append(p_idx)
    def precalc_explosion_masks(self):
        """预计算 3x3 炸弹掩码，保护外边界不被炸毁"""
        for y in range(1, HEIGHT - 1):
            for x in range(1, WIDTH - 1):
                idx = to_1d(x, y)
                mask = set()
                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                            n_idx = to_1d(nx, ny)
                            if n_idx not in self.boundary_walls:
                                mask.add(n_idx)
                self.explosion_masks[idx] = frozenset(mask)

    @functools.lru_cache(maxsize=1024)
    def get_maze_distances(self, walls: frozenset) -> dict:
        dist_table = {}
        has_bombs = bool(self.initial_state.bombs)
        for goal_idx in self.initial_goal_positions:
            dist_table[goal_idx] = {}
            pq = [(0, goal_idx)]
            visited = {goal_idx: 0}
            while pq:
                dist, curr = heapq.heappop(pq)
                if dist > visited.get(curr, float('inf')): continue

                dist_table[goal_idx][curr] = dist
                cx, cy = to_2d(curr)
                for dx, dy in DIRS:
                    # 假设把箱子从 (cx, cy) 拉到 (px, py)
                    # 根据推箱子物理，此时小车的发力站位必须是 (ppx, ppy)
                    px, py = cx - dx, cy - dy
                    ppx, ppy = px - dx, py - dy
                    if not (0 <= px < WIDTH and 0 <= py < HEIGHT): continue
                    if not (0 <= ppx < WIDTH and 0 <= ppy < HEIGHT): continue
                    p_idx = to_1d(px, py)
                    pp_idx = to_1d(ppx, ppy)
                    # 边界绝对不可逾越
                    if p_idx in self.boundary_walls or pp_idx in self.boundary_walls:
                        continue
                    step_cost = 1
                    is_p_wall = p_idx in walls
                    is_pp_wall = pp_idx in walls
                    # 如果箱子退回的地方有墙，或者小车站的地方有墙
                    if is_p_wall or is_pp_wall:
                        if not has_bombs:
                            continue  # 如果没有炸弹，墙绝对不可穿透，此路不通！
                        # 穿墙拉拽惩罚 (有几面墙就叠加几次惩罚)
                        if is_p_wall: step_cost += VIRTUAL_WALL_COST
                        if is_pp_wall: step_cost += VIRTUAL_WALL_COST

                    new_dist = dist + step_cost
                    if new_dist < visited.get(p_idx, float('inf')):
                        visited[p_idx] = new_dist
                        heapq.heappush(pq, (new_dist, p_idx))

        return dist_table
    def get_car_travel_distance(self, start_pos: int, target_pos: int, obs_set: FrozenSet[int]):
        """底层真实寻路 BFS"""
        if start_pos == target_pos: return 0
        queue = deque([(start_pos, 0)])
        visited = {start_pos}
        while queue:
            curr, dist = queue.popleft()
            cx, cy = to_2d(curr)
            for dx, dy in DIRS:  # 使用全局常量
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                    n_idx = to_1d(nx, ny)
                    if n_idx == target_pos:  # 提早退出检测
                        return dist + 1
                    if n_idx not in obs_set and n_idx not in visited:
                        visited.add(n_idx)
                        queue.append((n_idx, dist + 1))
        return float('inf')
     # maxsize=最大缓存条目
    @functools.lru_cache(maxsize=40000)
    def calc_heuristic(self, state: State) -> int:
        if not state.boxes or not state.goals: return 0

        dist_table = self.get_maze_distances(state.walls)
        # 按 ID 将箱子和目标进行分组
        boxes_by_id = defaultdict(list)
        goals_by_id = defaultdict(list)
        for b_pos, b_id in state.boxes: boxes_by_id[b_id].append(b_pos)
        for g_pos, g_id in state.goals: goals_by_id[g_id].append(g_pos)

        base_h = 0
        # 遍历每一类 ID，分别计算最小匹配代价
        for type_id in boxes_by_id.keys():
            b_list = boxes_by_id[type_id]
            g_list = goals_by_id.get(type_id, [])

            if not g_list:  # 如果没有匹配的目的地，说明死局或者炸弹逻辑改变，给个极大惩罚
                return 9999
            # KM 匈牙利算法
            cost_matrix = np.zeros((len(b_list), len(g_list)))
            for i, b_idx in enumerate(b_list):
                for j, g_idx in enumerate(g_list):
                    cost_matrix[i, j] = dist_table[g_idx].get(b_idx, 999)

            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            base_h += int(cost_matrix[row_ind, col_ind].sum())
            # =========================================================
            # 【新增：线性冲突惩罚 (Linear Conflict Penalty)】
            # 解决幽灵箱子互相穿透导致的乐观估算，强制避让惩罚
            # 1. 提取出 KM 算法刚刚算出的最优匹配对 (箱子坐标, 目标坐标)
            matched_pairs = []
            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] < 999:  # 排除死局
                    matched_pairs.append((b_list[r], g_list[c]))
            conflict_penalty = 0
            n_pairs = len(matched_pairs)
            # 2. 两两对比所有匹配好的箱子，寻找冲突
            for i in range(n_pairs):
                for j in range(i + 1, n_pairs):
                    b1_idx, g1_idx = matched_pairs[i]
                    b2_idx, g2_idx = matched_pairs[j]

                    b1_x, b1_y = to_2d(b1_idx)
                    g1_x, g1_y = to_2d(g1_idx)
                    b2_x, b2_y = to_2d(b2_idx)
                    g2_x, g2_y = to_2d(g2_idx)

                    # --- 行冲突 (Row Conflict) ---
                    # 条件：两个箱子和它们的终点，全部都在同一行！
                    if b1_y == b2_y == g1_y == g2_y:
                        # 判断是否交叉：B1 在 B2 左边，但 G1 却在 G2 右边
                        if (b1_x - b2_x) * (g1_x - g2_x) < 0:
                            # 发生错车冲突！至少需要 4 步的额外避让开销
                            conflict_penalty += 4
                    # --- 列冲突 (Column Conflict) ---
                    # 条件：两个箱子和它们的终点，全部都在同一列！
                    elif b1_x == b2_x == g1_x == g2_x:
                        # 判断是否交叉：B1 在 B2 上方，但 G1 却在 G2 下方
                        if (b1_y - b2_y) * (g1_y - g2_y) < 0:
                            conflict_penalty += 4
            # 将冲突惩罚叠加到基础启发值上
            base_h += conflict_penalty
        return base_h

    def is_deadlock(self, item_idx: int, walls: frozenset, item_positions: set, is_bomb: bool, n: int) -> bool:
        """
        O(1) 终极死锁检测：涵盖单角 L型死锁、2x2 多实体粘连死锁、走廊互相卡死
        :param item_idx: 当前刚被推入新位置的物体坐标
        :param walls: 静态墙体集合
        :param item_positions: 下一状态的所有实体（箱子+炸弹）坐标纯集合
        :param is_bomb: 当前推的是否为炸弹
        n: 现存tnt数量
        """
        ix, iy = to_2d(item_idx)
        def is_wall(x, y): #所有墙体
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT: return True
            idx = to_1d(x, y)
            return idx in walls
        def is_unbreakable(x, y):
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                return False
            idx = to_1d(x, y)
            return (idx in item_positions) or (idx in self.boundary_walls)
        def is_solid(x, y): #所有碰撞箱
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT: return True
            idx = to_1d(x, y)
            return (idx in walls) or (idx in item_positions)

        if n == 0 or (n == 1 and is_bomb):
            # 4.2 贴墙口袋死锁 (Pocket Deadlock) & 走廊连环卡死
            if is_unbreakable(ix + 1, iy) and is_solid(ix, iy + 1) and is_solid(ix + 1, iy + 1) and is_solid(ix,                                                                                                      iy - 1) and is_solid(
                ix + 1, iy - 1): return True
            if is_unbreakable(ix - 1, iy) and is_solid(ix, iy + 1) and is_solid(ix - 1, iy + 1) and is_solid(ix,
                iy - 1) and is_solid(ix - 1, iy - 1): return True
            if is_unbreakable(ix, iy + 1) and is_solid(ix + 1, iy) and is_solid(ix + 1, iy + 1) and is_solid(ix - 1,
                iy) and is_solid(ix - 1, iy + 1): return True
            if is_unbreakable(ix, iy - 1) and is_solid(ix + 1, iy) and is_solid(ix + 1, iy - 1) and is_solid(ix - 1,
                iy) and is_solid(ix - 1, iy - 1): return True
            
        # 2x2 实体粘连检测 (方形死锁)：任何情况都生效
        if is_unbreakable(ix + 1, iy) and is_unbreakable(ix, iy + 1) and is_unbreakable(ix + 1, iy + 1): return True
        if is_unbreakable(ix - 1, iy) and is_unbreakable(ix, iy + 1) and is_unbreakable(ix - 1, iy + 1): return True
        if is_unbreakable(ix + 1, iy) and is_unbreakable(ix, iy - 1) and is_unbreakable(ix + 1, iy - 1): return True
        if is_unbreakable(ix - 1, iy) and is_unbreakable(ix, iy - 1) and is_unbreakable(ix - 1, iy - 1): return True
        if n >= 2:
            return False
        k = 0
        if is_solid(ix + 1, iy) and is_solid(ix, iy + 1) and is_solid(ix + 1, iy + 1): k += 1
        if is_solid(ix - 1, iy) and is_solid(ix, iy + 1) and is_solid(ix - 1, iy + 1): k += 1
        if is_solid(ix + 1, iy) and is_solid(ix, iy - 1) and is_solid(ix + 1, iy - 1): k += 1
        if is_solid(ix - 1, iy) and is_solid(ix, iy - 1) and is_solid(ix - 1, iy - 1): k += 1
        if n == 0:
            #单角死锁检测 (L型墙角)
            w_U = is_wall(ix, iy - 1)
            w_D = is_wall(ix, iy + 1)
            w_L = is_wall(ix - 1, iy)
            w_R = is_wall(ix + 1, iy)
            if (w_U and w_L) or (w_U and w_R) or (w_D and w_L) or (w_D and w_R):
                return True
            if k > 0: return True
        return False

    def dfs_ida(self, current_state: State, g_score: int, threshold: int, path: List[MacroAction],current_h: int = -1):
        self.total_explored_nodes += 1

        if current_h == -1:
            current_h = self.calc_heuristic(current_state)

        f_score = g_score + self.current_weight * current_h
        # 【更新】：直接返回元组 (f, g, h)
        if len(path) > 150:
            return (float('inf'), 0, 0)
        if f_score > threshold:
            return (f_score, g_score, current_h)

        if not current_state.boxes: #成功
            self.solution_actions = path.copy()
            return -1

        # 【更新】：初始化记录最小越界节点的完整信息
        min_node_data = (float('inf'), 0, 0)

        box_positions = {p for p, _ in current_state.boxes}
        all_item_positions = box_positions | current_state.bombs
        obstacles = current_state.walls | all_item_positions
        current_goals_dict = {g_pos: g_id for g_pos, g_id in current_state.goals}

        movables = [(p, bid, False) for p, bid in current_state.boxes] + \
                   [(p, None, True) for p in current_state.bombs]

        # 【新增】：用于暂存本层所有的合法子节点，以便排序
        children = []

        for item_idx, item_id, is_bomb in movables:
            ix, iy = to_2d(item_idx)
            for dx, dy in DIRS:
                px, py = ix - dx, iy - dy
                nx, ny = ix + dx, iy + dy
                # 越界检测
                if not (0 <= px < WIDTH and 0 <= py < HEIGHT and 0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                    continue

                push_stand_idx = to_1d(px, py)
                next_item_idx = to_1d(nx, ny)
                #碰撞检测
                if push_stand_idx in obstacles and push_stand_idx != current_state.car_pos:
                    continue
                if next_item_idx in all_item_positions: continue

                exploded, consumed = False, False
                if next_item_idx in current_state.walls:
                    if is_bomb:
                        if next_item_idx in self.boundary_walls: continue
                        exploded = True
                    else:
                        continue
                elif not is_bomb:
                    target_goal_id = current_goals_dict.get(next_item_idx)
                    if target_goal_id is not None and target_goal_id == item_id:
                        consumed = True
                    elif not current_state.bombs and next_item_idx not in self.safe_mask:
                        continue
                # BFS寻路防线 (中等耗时，但在前面已经被筛掉了大量废动作)
                car_dist = self.get_car_travel_distance(current_state.car_pos, push_stand_idx, obstacles)
                if car_dist == float('inf'): continue

                # 分支可行 创建新state
                step_cost = car_dist + 1
                if exploded:
                    step_cost += BOMB_PENALTY  # 使用纯整数
                next_g_score = g_score + step_cost

                # 最耗时操作：内存分配与克隆 仅对幸存的分支执行
                next_walls = current_state.walls
                next_boxes = set(current_state.boxes)
                next_bombs = set(current_state.bombs)
                next_goals = set(current_state.goals)
                if exploded:
                    next_bombs.remove(item_idx)
                    next_walls = next_walls - self.explosion_masks[next_item_idx]
                elif consumed:
                    next_boxes.remove((item_idx, item_id))
                    next_goals.remove((next_item_idx, target_goal_id))
                else:
                    next_item_positions = (all_item_positions - {item_idx}) | {next_item_idx}
                    if self.is_deadlock(next_item_idx, current_state.walls, next_item_positions, is_bomb, len(next_bombs)):
                        continue
                    if is_bomb:
                        next_bombs.remove(item_idx)
                        next_bombs.add(next_item_idx)
                    else:
                        next_boxes.remove((item_idx, item_id))
                        next_boxes.add((next_item_idx, item_id))

                next_state = State(
                    car_pos=item_idx,
                    boxes=frozenset(next_boxes),
                    bombs=frozenset(next_bombs),
                    walls=next_walls,
                    goals=frozenset(next_goals)
                )
                # 查重
                if next_state in self.visited_states and self.visited_states[next_state] <= next_g_score:
                    continue

                next_h = self.calc_heuristic(next_state)
                next_f = next_g_score + self.current_weight * next_h
                action = MacroAction(item_idx, next_item_idx, push_stand_idx, exploded, consumed)
                children.append((next_f, next_h, next_state, next_g_score, action))

        # ---------------------------------------------------------
        # 【终极提速：节点排序 (Move Ordering)】
        # 按照 f 值从小到大排序。f 越小，说明这个动作越倾向于走向终点！

        children.sort(key=lambda x: (x[0], -x[3]))
        for next_f, next_h, next_state, next_g_score, action in children:

            # 如果即使是排在最前面的优选子节点也超过了阈值，直接跳过并更新门槛
            if next_f > threshold:
                if next_f < min_node_data[0]:
                    min_node_data = (next_f, next_g_score, next_h)
                break                                                  ####❌ #启用sort则改为break！！！！
            self.visited_states[next_state] = next_g_score
            path.append(action)

            # 携带算好的 next_h 传递给下一层，完美避免了重复的矩阵计算！
            res = self.dfs_ida(next_state, next_g_score, threshold, path, next_h)

            if res == -1: return -1
            # 【更新】：对比元组的第 0 项 (f_score)
            if res[0] < min_node_data[0]:
                min_node_data = res
            path.pop()
        return min_node_data

    def solve(self):
        """主入口：Weighted IDA* 迭代加深搜索"""
        self.current_weight = 3.5
        patience_limit =  N+1 # 每 N 次碰壁就衰减
        min_weight = 1.2
        base_threshold = self.calc_heuristic(self.initial_state)
        threshold = base_threshold * self.current_weight
        print(f"Initial Heuristic Threshold: {threshold}")

        iteration = 0  # 记录迭代深度
        while True:
            iteration += 1
            print(f"Iteration {iteration} | Searching with threshold: {threshold}...")

            # 记录本轮搜索前的总节点数 为了评估性能与动态调threshold
            nodes_before = self.total_explored_nodes

            self.visited_states.clear()
            self.visited_states[self.initial_state] = 0
            res = self.dfs_ida(self.initial_state, 0, threshold, [])

            if res == -1:
                return self.solution_actions
            if res[0] == float('inf'):  # 【更新】：检查元组第 0 项
                return None
            min_f, min_g, min_h = res
            # 1. 衰减判定：渐进二分衰减法 (Zeno's Dichotomy)
            if iteration % patience_limit == 0 and self.current_weight > min_weight:
                # 每次砍掉当前值与保底值之间一半的差距
                self.current_weight = min_weight + (self.current_weight - min_weight) * 0.5
                self.current_weight = round(self.current_weight, 1)

                # 如果因为精度问题导致逼近不动了，强制贴底
                if self.current_weight - min_weight < 0.2:
                    self.current_weight = min_weight
                print(f" Map is highly complex! Halving distance to min_weight: {self.current_weight:.2f}")
                threshold = min_g + self.current_weight * min_h
                patience_limit += patience_limit
                continue
            threshold = max(min_f, threshold + 4)
            # --- 核心泛化逻辑：基于吞吐量与迭代深度的动态阈值 ---
            nodes_this_round = self.total_explored_nodes - nodes_before
            '''if nodes_this_round < 1500:
                adaptive_step = min(10, 2 + int(iteration ** 0.5))
                threshold = max(min_f, threshold + adaptive_step)
            elif nodes_this_round > 30000:
                threshold = max(min_f, threshold + 4)
            else:
                threshold = max(min_f, threshold + 4)'''


# 测试执行部分
# ---------------------------------------------------------
if __name__ == "__main__":
    solver = SokobanSolver(m150)  # 这里替换为你想要测试的地图

    print("🚀 引擎启动，开始探索最佳路径...\n")

    # --- 启动性能追踪 ---
    tracemalloc.start()
    start_time = time.perf_counter()

    # 执行求解
    actions = solver.solve()

    # --- 停止性能追踪 ---
    end_time = time.perf_counter()
    current_mem, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    # 计算指标
    execution_time = end_time - start_time
    peak_mem_mb = peak_mem / (1024 * 1024)

    # --- 打印结果与指令集 ---
    if actions:
        print("\n" + "=" * 40)
        print("🎯 最终战术指令集 (最优解)")
        print("=" * 40)
        for i, act in enumerate(actions):
            action_type = "💣 炸毁墙体" if act.is_bomb_exploded else (
                "✅ 箱子入列消除" if act.is_consumed else "➡️ 平移物体")
            print(
                f"Step {i + 1:02d}: {action_type} - 从 {to_2d(act.item_start_idx)} 推至 {to_2d(act.item_end_idx)} (小车站位: {to_2d(act.car_stand_pos)})")
    else:
        print("\n❌ 求解失败：无可行解或所有路径已死锁。")

    # --- 打印性能基准测试报告 ---
    print("\n" + "=" * 40)
    print(" 性能基准测试报告 (Python 验证环境)")
    print("=" * 40)
    print(f" 寻路耗时      : {execution_time:.4f} 秒")
    print(f" 峰值内存占用  : {peak_mem_mb:.4f} MB")
    print(f" step = {len(actions)}")
    if hasattr(solver, 'total_explored_nodes'):
        print(f" 累计展开节点数: {solver.total_explored_nodes:,} 个")
        print(f" 节点处理速度  : {int(solver.total_explored_nodes / execution_time):,} 节点/秒")
    print("=" * 40)