"""

"""

import os
import sys
import math
import heapq
import numpy as np
import cv2
from tsp_solver.greedy import solve_tsp
import copy
from tqdm import tqdm

import config
from baiduMap import baidu_map

class Env:
    """
    Env 2D
    """
    def __init__(self,outpool_points):
        self.x_range = 51  # size of background
        self.y_range = 31
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        # self.motions = [(-2, 0), (-2, 2), (0, 2), (2, 2),
        #                 (2, 0), (2, -2), (0, -2), (-2, -2)]
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

    while current_y <= (y + h):
        while current_x <= (x + w) and current_x >= x:
            point = (current_x, current_y)
            in_cnt = cv2.pointPolygonTest(contour, point, True)
            if in_cnt <= safe_distance:
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


def distance(p0, p1, digits=2):
    a = map(lambda x: (x[0] - x[1]) ** 2, zip(p0, p1))
    return round(math.sqrt(sum(a)), digits)

safe_distance=0

# 判断轨迹是否经过陆地区域
def cross_outpool(point_i,point_j,pool_cnts):
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
    if steps==0:
        return True
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
        in_cnt = cv2.pointPolygonTest(pool_cnts, point_temp, True)
        if in_cnt < safe_distance:
            # 经过湖泊周围陆地
            return False
    # 不经过陆地
    return True


# path matrix
path_matrix = {}

# 统计点之间距离
def measure_distance(scan_cnt,pool_cnt,outpool_set,map_connect):
    global path_matrix
    l = len(scan_cnt)
    distance_matrix = np.full(shape=(l, l),fill_value=np.inf)
    for i in tqdm(range(l)):
        for j in range(l):
            if i == j:
                distance_matrix[i, j] = 0
            if i > j :
                continue
            d = distance(scan_cnt[i], scan_cnt[j], digits=2)
            distance_matrix[i,j] =d
            distance_matrix[j,i] =d

    for i in tqdm(range(l)):
        c = min(len(scan_cnt), map_connect)
        d = copy.deepcopy(distance_matrix[i, :])
        sort_scan_cnt = list(d)
        sort_scan_cnt.sort()
        end_scan_index_list = [sort_scan_cnt.index(i) for i in sort_scan_cnt[1:1 + c]]
        for j in range(l):
            if i == j:
                distance_matrix[i, j] = 0
                continue
            if i > j:
                continue
            if j in end_scan_index_list and not cross_outpool(scan_cnt[i],scan_cnt[j],pool_cnt):
                print(i,'-->',j,'False')
                s_index = i
                e_index = j
                s_start = (scan_cnt[s_index][0], scan_cnt[s_index][1])
                s_goal = (scan_cnt[e_index][0], scan_cnt[e_index][1])
                # 去不了会搜索报错
                try:
                    astar = AStar(s_start, s_goal, "euclidean", outpool_set)
                    path, visited = astar.searching()
                    distance_i_j = 0
                    for index_i, value in enumerate(path):
                        if index_i < len(path) - 1:
                            distance_i_j += distance(value, path[index_i + 1])
                    distance_matrix[i, j] = distance_i_j
                    distance_matrix[j, i] = distance_i_j
                    path_matrix.update({'%d_%d' % (i, j): path[::-1]})
                except:
                    print('error searching',i,'-->',j,'False')
                    distance_matrix[i, j] = math.inf
                    distance_matrix[j, i] = math.inf
            elif not cross_outpool(scan_cnt[i],scan_cnt[j],pool_cnt):
                distance_matrix[i, j] = math.inf
                distance_matrix[j, i] = math.inf
            else:
                dis = distance(scan_cnt[i], scan_cnt[j], digits=2)
                distance_matrix[i,j] =dis
                distance_matrix[j,i] =dis
            if not cross_outpool(scan_cnt[i],scan_cnt[j],pool_cnt):
                print('i,j',i,j)
                distance_matrix[i, j] = math.inf
                distance_matrix[j, i] = math.inf

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

#判断points内的点处在同一条直线上吗？
#points内至少有3个点。
def on_one_line(points):
    delta_x = points[1][0] - points[0][0]
    delta_y = points[1][1] - points[0][1]
    distance_square = delta_x **2 + delta_y **2
    # 传入了相同的点 返回True
    if distance_square==0:
        return True
    sin_times_cos = delta_x * delta_y/ distance_square
    for j in range(2, len(points)):
        dx = points[j][0] - points[0][0]
        dy = points[j][1] - points[0][1]
        if math.fabs(dx * dy / (dx * dx + dy * dy) - sin_times_cos) > 10 ** -9:
            return False
    return True

# 将直线上多个点合并为按直线最少的点
def multi_points_to_simple_points(points):
    if len(points)<=3:
        return points
    else:
        return_points=[]
        test_points = []
        return_points.append(points[0])
        test_points.append(points[0])
        test_points.append(points[1])
        # test_points.append(points[2])
        for index_i in range(2,len(points)):
            test_points.append(points[index_i])
            if on_one_line(test_points):
                pass
            else:
                # if index_i == len(points) - 1:
                return_points.append(test_points[2])
            test_points.pop(0)
        return return_points

def get_path(baidu_map_obj=None,
             mode=0,
             target_lng_lats=None,
             target_pixs=None,
             b_show=False,
             back_home=False,
             map_connect = 7,
             pix_gap=30):
    """
    根据设置模式返回高德地图上规划路径
    :param baidu_map_obj 地图对象
    :param mode 选择模式
    :param target_lng_lats 目标经纬度集合，传入为高德经纬度
    :param target_pixs 目标像素
    :param b_show 是否显示图像
    :param map_connect 搜索一个点最多连接数量
    :param pix_gap 自动搜索像素间隔
    mode
    ０　到达目标点后停留
    １　到达多个目标点
    ２　扫描整个湖泊
    4  返航
    """
    global path_matrix
    if baidu_map_obj==None:
        baidu_map_obj = baidu_map.BaiduMap(config.init_gaode_gps, zoom=16, scale=1, map_type=baidu_map.MapType.gaode)
        pool_cnts,(pool_cx,pool_cy) = baidu_map_obj.get_pool_pix(b_show=False)
        if pool_cnts is None:
            return 'pool_cx is None'
    baidu_map_obj.outpool_cnts_set = get_outpool_set(np.array(baidu_map_obj.pool_cnts))

    # 无GPS调试模式 以湖泊中心作为起点
    if config.home_debug:
        baidu_map_obj.ship_gaode_lng_lat=config.init_gaode_gps
        baidu_map_obj.ship_gps=config.init_gaode_gps
    if baidu_map_obj.ship_gps is None:
        return 'no ship gps'
    if mode==0:
        if target_lng_lats is None:
            return 'target_pixs is None'
        elif len(target_lng_lats)>1:
            return 'len(target_pixs) is >1 choose mode 1'
        if baidu_map_obj.ship_pix is None :
            if baidu_map_obj.ship_gaode_lng_lat is None :
                baidu_map_obj.ship_gaode_lng_lat = baidu_map_obj.gps_to_gaode_lng_lat(baidu_map_obj.ship_gps)
            baidu_map_obj.ship_pix = baidu_map_obj.gaode_lng_lat_to_pix(baidu_map_obj.ship_gaode_lng_lat)
        s_start = tuple(baidu_map_obj.ship_pix)
        s_goal = tuple(baidu_map_obj.gaode_lng_lat_to_pix(target_lng_lats[0]))
        print('s_start,s_goal',s_start,s_goal)
        # TODO 测试使用间隔两个像素搜索
        # s_start = list(s_start)
        # s_goal = list(s_goal)
        #
        # if s_start[0]%2!=0:
        #     s_start[0] = s_start[0]+1
        # if s_start[1]%2!=0:
        #     s_start[1] = s_start[0]+1
        # if s_goal[0]%2!=0:
        #     s_goal[0] = s_goal[0]+1
        # if s_goal[1]%2!=0:
        #     s_goal[1] = s_goal[1]+1
        # s_start = tuple(s_start)
        # s_goal = tuple(s_goal)
        # print('s_start,s_goal', s_start, s_goal)
        # 判断是否能直线到达，不能则采用路径搜索
        if not cross_outpool(s_start,s_goal,baidu_map_obj.pool_cnts):
            astar = AStar(s_start, s_goal, "euclidean", baidu_map_obj.outpool_cnts_set)
            try:
                astar_path, visited = astar.searching()
                print('astar_path', astar_path)
                return_pix_path = astar_path[::-1]
                # 返航时添加
                if back_home:
                    return_pix_path.append(astar_path)
                print('原始长度',len(return_pix_path))
                return_pix_path = multi_points_to_simple_points(return_pix_path)
                print('简化后长度', len(return_pix_path))
                _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
                if b_show:
                    baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img,
                                                           [np.array(astar_path, dtype=np.int32)], False, (255, 0, 0),
                                                           1)
                    cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 0,255], -1)
                    cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 0,255], -1)
                    baidu_map_obj.show_img = cv2.drawContours(baidu_map_obj.show_img, [return_pix_path], -1,
                                                              (0, 0, 255), 3)
                    cv2.imshow('scan', baidu_map_obj.show_img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                return return_gaode_lng_lat_path
            except Exception as e :
                print('error ',e)
                if b_show:
                    cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 255, 0], -1)
                    cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 0, 255], -1)
                    cv2.imshow('scan', baidu_map_obj.show_img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                return 'can not fand path'
        # 直接可达模式
        else:
            return_pix_path = []
            return_pix_path.append(s_start)
            return_pix_path.append(s_goal)
            _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
            if b_show:
                cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 255, 0], -1)
                cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 255, 0], -1)
                baidu_map_obj.show_img = cv2.drawContours(baidu_map_obj.show_img, np.array([return_pix_path]), -1,
                                                          (0, 0, 255), 3)
                cv2.imshow('scan', baidu_map_obj.show_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            # 返航时添加家的经纬度
            if back_home:
                if config.home_debug:
                    return_gaode_lng_lat_path.append(config.init_gaode_gps)
                else:
                    return_gaode_lng_lat_path.append(baidu_map_obj.init_ship_gaode_lng_lat)
            return return_gaode_lng_lat_path

    elif mode == 1:
        if target_lng_lats is None:
            return 'target_pixs is None'
        elif len(target_lng_lats) <= 1:
            return 'len(target_pixs) is<=1 choose mode 0'
        if baidu_map_obj.ship_pix is None:
            if baidu_map_obj.ship_gaode_lng_lat is None:
                baidu_map_obj.ship_gaode_lng_lat = baidu_map_obj.gps_to_gaode_lng_lat(baidu_map_obj.ship_gps)
            baidu_map_obj.ship_pix = baidu_map_obj.gaode_lng_lat_to_pix(baidu_map_obj.ship_gaode_lng_lat)

        target_pixs=[]
        for target_lng_lat in target_lng_lats:
            target_pixs.append(baidu_map_obj.gaode_lng_lat_to_pix(target_lng_lat))
        target_pixs.insert(0,baidu_map_obj.ship_pix)
        distance_matrix = measure_distance(target_pixs,baidu_map_obj.pool_cnts,baidu_map_obj.outpool_cnts_set,map_connect=config.find_points_num)
        if back_home:
            tsp_path = solve_tsp(distance_matrix, endpoints=(0, 0))
        else:
            tsp_path = solve_tsp(distance_matrix, endpoints=(0, (len(target_pixs) - 1)))
        path_points=[]
        print('path_matrix',path_matrix)
        for index_i,val in enumerate(tsp_path):
            if index_i < len(tsp_path) - 1:
                if '%d_%d'%(val,tsp_path[index_i+1]) in path_matrix.keys():
                    path_points.extend(path_matrix['%d_%d'%(val,tsp_path[index_i+1])])
                elif '%d_%d'%(tsp_path[index_i+1],val) in path_matrix.keys():
                    path_points.extend(path_matrix['%d_%d' % (tsp_path[index_i + 1],val)][::-1])
                else:
                    path_points.append(target_pixs[val])
            elif index_i == len(tsp_path) - 1:
                if '%d_%d'%(val,tsp_path[index_i-1]) in path_matrix.keys() or '%d_%d'%(val,tsp_path[index_i-1]) in path_matrix.keys():
                    pass
                else:
                    path_points.append(target_pixs[val])

        return_pix_path = path_points
        print('原始长度', len(return_pix_path))
        return_pix_path = multi_points_to_simple_points(return_pix_path)
        print('简化后长度', len(return_pix_path))

        _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
        if b_show:
            baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img, [np.array(path_points, dtype=np.int32)],
                                                   False, (255, 0, 0), 1)
            cv2.imshow('scan', baidu_map_obj.show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return return_gaode_lng_lat_path

    elif mode == 2:
        baidu_map_obj.scan_pool(baidu_map_obj.pool_cnts, pix_gap=pix_gap, b_show=False)
        print('len(scan_cnt)', len(baidu_map_obj.scan_point_cnts))
        distance_matrix = measure_distance(baidu_map_obj.scan_point_cnts,baidu_map_obj.pool_cnts,baidu_map_obj.outpool_cnts_set,map_connect=map_connect)
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
                    pass
                    # path_points.append(baidu_map_obj.scan_point_cnts[val])
        if b_show:
            baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img, [np.array(path_points, dtype=np.int32)],
                                                   False, (255, 0, 0), 1)
            cv2.imshow('scan', baidu_map_obj.show_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return_pix_path = path_points
        return_pix_path = path_points
        print('原始长度', len(return_pix_path))
        return_pix_path = multi_points_to_simple_points(return_pix_path)
        print('简化后长度', len(return_pix_path))
        _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
        return return_gaode_lng_lat_path

    elif mode == 3:
        pass

    # back home
    elif mode == 4:
        if baidu_map_obj.init_ship_gps is None:
            return 'ship init gps is None'
        if baidu_map_obj.init_ship_pix is None:
            if baidu_map_obj.init_ship_gaode_lng_lat is None:
                baidu_map_obj.init_ship_gaode_lng_lat= baidu_map_obj.gps_to_gaode_lng_lat(baidu_map_obj.init_ship_gps)
            baidu_map_obj.init_ship_pix = baidu_map_obj.gaode_lng_lat_to_pix(baidu_map_obj.init_ship_gaode_lng_lat)

        baidu_map_obj.ship_gaode_lng_lat = baidu_map_obj.gps_to_gaode_lng_lat(baidu_map_obj.ship_gps)
        baidu_map_obj.ship_pix = baidu_map_obj.gaode_lng_lat_to_pix(baidu_map_obj.ship_gaode_lng_lat)

        s_start = tuple(baidu_map_obj.ship_pix)
        s_goal = tuple(baidu_map_obj.init_ship_pix)
        print('s_start,s_goal', s_start, s_goal)
        # 判断是否能直线到达，不能则采用路径搜索
        if not cross_outpool(s_start, s_goal, baidu_map_obj.pool_cnts):
            astar = AStar(s_start, s_goal, "euclidean", baidu_map_obj.outpool_cnts_set)
            # try:
            astar_path, visited = astar.searching()
            print('astar_path', astar_path)
            baidu_map_obj.show_img = cv2.polylines(baidu_map_obj.show_img, [np.array(astar_path, dtype=np.int32)],
                                                   False, (255, 0, 0), 1)
            return_pix_path = astar_path[::-1]
            return_pix_path = multi_points_to_simple_points(return_pix_path)
            _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
            if b_show:
                cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 0, 255], -1)
                cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 0, 255], -1)
                baidu_map_obj.show_img = cv2.drawContours(baidu_map_obj.show_img, [return_pix_path], -1, (0, 0, 255), 3)
                cv2.imshow('scan', baidu_map_obj.show_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            return return_gaode_lng_lat_path
            # except:
            #     if b_show:
            #         cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 255, 0], -1)
            #         cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 255, 0], -1)
            #         baidu_map_obj.show_img = cv2.drawContours(baidu_map_obj.show_img, [return_pix_path], -1, (0, 0, 255), 3)
            #
            #         cv2.imshow('scan', baidu_map_obj.show_img)
            #         cv2.waitKey(0)
            #         cv2.destroyAllWindows()
            #         return -3
        # 直接可达模式
        else:
            return_pix_path = []
            return_pix_path.append(s_start)
            return_pix_path.append(s_goal)
            _, return_gaode_lng_lat_path = baidu_map_obj.pix_to_gps(return_pix_path)
            if b_show:
                cv2.circle(baidu_map_obj.show_img, s_start, 5, [255, 255, 0], -1)
                cv2.circle(baidu_map_obj.show_img, s_goal, 5, [255, 255, 0], -1)
                cv2.imshow('scan', baidu_map_obj.show_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            return return_gaode_lng_lat_path
    else:
        return 1

if __name__ == '__main__':
    # 114.431299,30.521363
    # 114.433853,30.519553
    # [114.431133,30.522252],[114.432464,30.521108],[114.430983,30.519953],[114.432625,30.52036],[114.430726,30.519158],[114.430726,30.519158],[114.433853,30.519553]
    r = get_path(mode=0,b_show=True,target_lng_lats=[[114.431299,30.521363]])
    # r = get_path(mode=2,b_show=True,pix_gap=150)
    print('r',r)

# baidu_map_obj.ship_pix [566, 565]
# (x, y, w, h) (420, 249, 414, 653)
