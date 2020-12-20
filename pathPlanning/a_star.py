"""

"""

import os
import sys
import math
import heapq
import numpy as np
import cv2
from tsp_solver.greedy import solve_tsp

from baiduMap.baidu_map import BaiduMap

class Env:
    """
    Env 2D
    """
    def __init__(self,outpool_points):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.outpool_points = outpool_points
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """
        obs = set()
        for i in self.outpool_points:
            obs.add((i[0],i[1]))
        return obs



class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type,outpool_points):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = Env(outpool_points)  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])


def get_outpool_set(contour,safe_distance=0):
    """
    :param contour 湖泊轮廓
    :param safe_distance 像素安全距离
    """
    # 求坐标点最大外围矩阵
    (x, y, w, h) = cv2.boundingRect(contour)
    print('(x, y, w, h)', (x, y, w, h))
    # 循环判断点是否在湖泊范围外
    outpool_cnts_set = []
    # 间距
    pix_gap = 1
    # 起始点
    start_x, start_y = x, y + pix_gap
    # 当前点
    current_x, current_y = start_x, start_y
    # 判断x轴是递增的加还是减 True 为加
    b_add_or_sub = True

    while current_y < (y + h):
        while current_x <= (x + w) and current_x >= x:
            point = (current_x, current_y)
            in_cnt = cv2.pointPolygonTest(contour, point, True)
            # print('in cnt',in_cnt)
            if in_cnt < safe_distance:
                # print(in_cnt,point)
                outpool_cnts_set.append(list(point))
            if b_add_or_sub:
                current_x += pix_gap
            else:
                current_x -= pix_gap
        current_y += pix_gap
        if b_add_or_sub:
            current_x -= pix_gap
            b_add_or_sub = False
        else:
            current_x += pix_gap
            b_add_or_sub = True
    return outpool_cnts_set


import math

def distance(p0, p1, digits=2):
    a = map(lambda x: (x[0] - x[1]) ** 2, zip(p0, p1))
    return round(math.sqrt(sum(a)), digits)

safe_distance=1

# 判断轨迹是否经过陆地区域
def cross_outpool(point_i,point_j,pool_cnt):
    line_points = []
    dx = point_j[0] - point_i[0]
    dy = point_j[1] - point_i[1]
    steps = 0
    # 斜率判断
    if abs(dx) > abs(dy):
        steps = abs(dx)
    else:
        steps = abs(dy)
    # 必有一个等于1，一个小于1
    delta_x = float(dx / steps)
    delta_y = float(dy / steps)

    # 四舍五入，保证x和y的增量小于等于1，让生成的直线尽量均匀
    x = point_i[0] + 0.5
    y = point_i[1] + 0.5
    for i in range(0, int(steps + 1)):
        # 绘制像素点
        line_points.append([int(x), int(y)])
        x += delta_x
        y += delta_y
    for point in line_points:
        point_temp = (point[0], point[1])
        in_cnt = cv2.pointPolygonTest(pool_cnt, point_temp, True)
        # print('in cnt',in_cnt)
        if in_cnt < safe_distance:
            return False
    return True

from tqdm import tqdm

# path matrix
path_matrix = {}

# 统计点之间距离
def measure_distance(scan_cnt,pool_cnt,outpool_set):
    global path_matrix
    l = len(scan_cnt)
    distance_matrix = np.full(shape=(l, l),fill_value=np.inf)

    for i in tqdm(range(l)):
        for j in range(l):
            if i==j:
                distance_matrix[i, j] = 0
                continue
            if i>j:
                continue
            if not cross_outpool(scan_cnt[i],scan_cnt[j],pool_cnt):
                print(i,'-->',j,'False')
                s_index = i
                e_index = j
                s_start = (scan_cnt[s_index][0], scan_cnt[s_index][1])
                s_goal = (scan_cnt[e_index][0], scan_cnt[e_index][1])
                astar = AStar(s_start, s_goal, "euclidean", outpool_set)
                path, visited = astar.searching()

                # print('s_start', s_start)
                # print('s_goal', s_goal)
                # print('path', path)
                # print('visited', visited)
                distance_i_j = 0
                for index_i, value in enumerate(path):
                    if index_i < len(path) - 1:
                        distance_i_j += distance(value, path[index_i + 1])
                # print('distance_i_j', distance_i_j)
                distance_matrix[i, j] = distance_i_j
                distance_matrix[j, i] = distance_i_j
                path_matrix.update({'%d_%d'%(i,j):path[::-1]})
            else:
                d = distance(scan_cnt[i], scan_cnt[j], digits=2)
                distance_matrix[i,j] =d
                distance_matrix[j,i] =d
    return distance_matrix


def return_to_base(point1,point2,baidu_map_obj,b_show=False):
    """
    return to start point
    """
    plus=1000
    s_start =tuple([int(point1[0]*plus),int(point1[1]*plus)])
    s_goal = tuple([int(point2[0]*plus),int(point2[1]*plus)])
    _,outpool_lng_lats_set = baidu_map_obj.pix_to_gps(baidu_map_obj.outpool_cnts_set)
    baidu_map_obj.outpool_lng_lats_set = [[int(i[0]*plus),int(i[1]*plus)] for i in outpool_lng_lats_set]
    in_cnt_start = cv2.pointPolygonTest(np.array(baidu_map_obj.outpool_lng_lats_set), s_start, True)
    in_cnt_goal = cv2.pointPolygonTest(np.array(baidu_map_obj.outpool_lng_lats_set), s_goal, True)
    print('in_cnt_start', in_cnt_start)
    print('in_cnt_goal', in_cnt_goal)
    astar = AStar(s_start, s_goal, "euclidean", baidu_map_obj.outpool_lng_lats_set)
    astar_path, visited = astar.searching()
    print('astar_path', astar_path)
    return astar_path

def get_path(baidu_map_obj=None,mode=1,b_show=False,map_connect = 1):
    """
    param mode 模式　
    ０　到达目标点后停留
    １　到达目标点后返回
    ２　扫描整个湖泊
    """
    point1 = [114.391098, 30.572314]
    point2 = [114.370186, 30.554962]
    point3 = [114.398213, 30.54501]
    if baidu_map_obj==None:

        baidu_map_obj = BaiduMap([114.390129,30.559005],zoom=14)
        # obj = BaiduMap([114.411257,30.58388],zoom=14)
        # obj = BaiduMap([114.431954,30.562239],zoom=14)
        # obj = BaiduMap([114.443596,30.545694],zoom=14)
        pool_cnts,(pool_cx,pool_cy) = baidu_map_obj.get_pool_pix(b_show=False)
        if pool_cnts is None:
            return pool_cx
    baidu_map_obj.scan_pool(baidu_map_obj.pool_cnts,pix_gap=50,b_show=False)
    print('len(scan_cnt)',len(baidu_map_obj.scan_point_cnts))
    baidu_map_obj.outpool_cnts_set = get_outpool_set(np.array(baidu_map_obj.scan_point_cnts))

    if mode==0:
        s_index = 0
        e_index = 10
        print('s e ', baidu_map_obj.scan_point_cnts[s_index], baidu_map_obj.scan_point_cnts[e_index])
        s_start = tuple(baidu_map_obj.scan_point_cnts[s_index])
        s_goal = tuple(baidu_map_obj.scan_point_cnts[e_index])
        astar = AStar(s_start, s_goal, "euclidean", baidu_map_obj.outpool_cnts_set)
        astar_path, visited = astar.searching()
        print('astar_path', astar_path)
        baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img, [np.array(astar_path, dtype=np.int32)], False, (255, 0, 0), 1)
        return_pix_path = astar_path
    else :
        distance_matrix = measure_distance(baidu_map_obj.scan_point_cnts,baidu_map_obj.pool_cnts,baidu_map_obj.outpool_cnts_set)
        print(distance_matrix.shape)
        print('distance_matrix',np.array(distance_matrix))
        tsp_path = solve_tsp(distance_matrix,endpoints=(0,0))
        print('tsp_path',tsp_path)
        path_points = []
        print('baidu_map_obj.pool_cnts', baidu_map_obj.pool_cnts)
        print('path_matrix',path_matrix)
        print('list path_matrix',list(path_matrix.keys()))
        for index_i,val in enumerate(tsp_path):
            if index_i < len(tsp_path) - 1:
                if '%d_%d'%(val,tsp_path[index_i+1]) in path_matrix.keys():
                    path_points.extend(path_matrix['%d_%d'%(val,tsp_path[index_i+1])])
                elif '%d_%d'%(tsp_path[index_i+1],val) in path_matrix.keys():
                    path_points.extend(path_matrix['%d_%d' % (tsp_path[index_i + 1],val)][::-1])
                else:
                    path_points.append(baidu_map_obj.scan_point_cnts[val])
            elif index_i == len(tsp_path) - 1:
                if '%d_%d'%(val,tsp_path[index_i-1]) in path_matrix.keys() or '%d_%d'%(val,tsp_path[index_i-1]) in path_matrix.keys():
                    pass
                else:
                    path_points.append(baidu_map_obj.scan_point_cnts[val])
        baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img, [np.array(path_points, dtype=np.int32)], False, (255, 0, 0), 1)
        return_pix_path = path_points

    return_to_base_path = return_to_base(point3,point2,baidu_map_obj)
    print('return_to_base_path',return_to_base_path)
    print('len(return_pix_path)', len(return_pix_path))
    return_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
    print('return_lng_lat_path', return_lng_lat_path)

    if b_show:
        cv2.imshow('scan', baidu_map_obj.show_img )
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return return_lng_lat_path


if __name__ == '__main__':
    get_path(b_show=True)
