import numpy as np
from numpy import flipud
from random import sample

class Map():
    def __init__(self, map: np.ndarray, exits: list, start_points: list, random_exits:list, name, radius, map_spawn: np.ndarray):
        # 对地图进行翻转操作
        map = flipud(map)
        self.map = map.T
        map_spawn = flipud(map_spawn)
        self.map_spawn = map_spawn.T
        self.exits = exits
        self.start_points = start_points
        self.radius = radius
        self.create_radius = self.radius
        self.name = name
        self.random_exits = random_exits
        self.row = self.map.shape[0]
        self.col = self.map.shape[1]

    def get_render_scale(self, window_size: int = 500):
        '''
        得到缩放比例，500*500的窗口大小
        :return:
        '''
        size = self.map.shape[0]
        return window_size / size

    def get_random_exit(self, index):
        return sample(self.random_exits[index], 1)[0]

    def __str__(self):
        return self.name

start_points_map1 = [(12.5, 7.5), (10.5, 5.5), (7.5, 7.5), (12.5, 12.5), (10.5, 14.5), (7.5, 12.5)]
exit_map1 = [(19.5, 7), (9, 0.5), (0.5, 7), (19.5, 12), (9, 19.5), (0.5, 12)]
radius_map1 = 1
random_exits1 = [[3], [4], [5], [6], [7], [8]]

#20*20
map1 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 7, 7, 7, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6],
    [8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6],
    [8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6],
    [2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2],
    [2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2]
])

start_points_map2 = [(1.5, 3), (1.5, 8)]
exit_map2 = [(11.5, 2), (11.5, 9)]
radius_map2 = 1
random_exits2 = [[3], [4]]

#10*10
map2 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])
#map_2对应的生成点地图
map2_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

start_points_map5 = [(1.5, 3), (1.5, 8)]
exit_map5 = [(11.5, 2), (11.5, 9)]
radius_map5 = 1
random_exits5 = [[3], [4]]

#不进行交叉
map5 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

map5_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 1, 1, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 1, 1, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 1, 1, 4, 4, 4, 0, 0, 0, 2],
    [2, 3, 3, 1, 1, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 1, 1, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 1, 1, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

start_points_map6 = [(1.5, 3), (1.5, 8)]
exit_map6 = [(11.5, 2), (11.5, 9)]
radius_map6 = 1
random_exits6 = [[3], [4]]

map6 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 4],
    [2, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

map6_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 2],
    [2, 4, 4, 1, 1, 4, 4, 1, 1, 0, 0, 2],
    [2, 4, 4, 1, 1, 4, 4, 1, 1, 0, 0, 2],
    [2, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 1, 1, 3, 3, 1, 1, 0, 0, 2],
    [2, 3, 3, 1, 1, 3, 3, 1, 1, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 2],
    [2, 3, 3, 3, 3, 3, 3, 3, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

start_points_map7 = [(7.5, 11.5), (11.5, 11.5), (7.5, 8.5), (11.5, 8.5)]
exit_map7 = [(4.5, 19.5), (16.5, 19.5), (4.5, 0.5), (16.5, 0.5)]
radius_map7 = 1
random_exits7 = [[3], [4], [5], [6]]

#20*20
map7 = np.array([
    [2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 5, 5, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 6, 6, 6, 2, 2]
])

start_points_map8 = [(2, 5)]
exit_map8 = [(2, 11.5),(9, 11.5)]
radius_map8 = 1.5
random_exits8 = [[3, 4]]

map8 = np.array([
    [2, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 2],
    [2, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 2],
    [2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 2],
    [2, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 2],
    [2, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

start_points_map9 = [(3.5, 10.5), (17.5, 9.5)]
exit_map9 = [(8.5, 19.5), (8.5, 0.5)]
radius_map9 = 1.5
random_exits9 = [[3], [4]]

#20*20
map9 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
])

start_points_map10 = [(8.5, 6.5),(8.5, 8.5),(6.5, 6.5),(6.5, 8.5)]
exit_map10 = [(14.5, 5.5),(14.5, 10.5),(0.5, 5.5),(0.5, 10.5)]
radius_map10 = 1.5
random_exits10 = [[3], [4], [5], [6]]

map10 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

map10_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 1, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 6, 6, 6, 6, 4, 4, 4, 4, 4, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 0, 0, 5, 5, 5, 5, 3, 3, 3, 3, 3, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

start_points_map11 = [(5.5, 5.5),(9.5, 5.5),(5.5, 8.5),(9.5, 8.5)]
exit_map11 = [(2.5, 0.5),(12.5, 0.5),(0.5, 7.5),(12.5, 14.5)]
radius_map11 = 5
random_exits11 = [[3], [4], [5], [6]]

map11 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 6, 6, 6, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [5, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2],
])

map11_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 5, 5, 5, 1, 1, 1, 1, 1, 1, 1, 6, 6, 6, 2],
    [2, 5, 5, 5, 1, 1, 1, 1, 1, 1, 1, 6, 6, 6, 2],
    [2, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 2],
    [2, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 2],
    [2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 2],
    [2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 2],
    [2, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 2],
    [2, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 4, 4, 4, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 3, 3, 3, 3, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

#random_init_mode基本不使用start_points、radius_map、random_exits,只要数量和行人种类一致就可以了，行人生成位置由map_spawn矩阵生成的start_point_dic决定 exit_map要注意
start_points_map12 = [(1, 1),(1, 1),(1, 1),(1, 1)]
exit_map12 = [(14.5, 5.5),(14.5, 10.5),(0.5, 5.5),(0.5, 10.5)]
radius_map12 = 1.5
random_exits12 = [[3], [4], [5], [6]]

#2墙，1障碍物，大于等于3的数字表示出口
map12 = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [6, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 4],
    [6, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 4],
    [6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2],
    [5, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

map12_spawn = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 6, 6, 6, 4, 4, 4, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 6, 6, 6, 4, 4, 4, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 6, 6, 6, 4, 4, 4, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 6, 6, 6, 4, 4, 4, 1, 1, 0, 2],
    [2, 0, 0, 0, 0, 6, 6, 6, 4, 4, 4, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 6, 6, 6, 4, 4, 4, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 5, 5, 5, 3, 3, 3, 1, 1, 0, 2],
    [2, 0, 0, 0, 0, 5, 5, 5, 3, 3, 3, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 5, 5, 5, 3, 3, 3, 0, 0, 0, 2],
    [2, 0, 0, 1, 1, 5, 5, 5, 3, 3, 3, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 5, 5, 5, 3, 3, 3, 1, 1, 0, 2],
    [2, 0, 0, 1, 1, 5, 5, 5, 3, 3, 3, 1, 1, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

# start_points_map12 = [(2.5, 7.5),(12.5, 2.5)]
# exit_map12 = [(2.5, 0.5),(7.5, 14.5)]
# radius_map12 = 1
# random_exits12 = [[3], [4]]
#
# map12 = np.array([
#     [2, 2, 2, 2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 2, 2],
#     [2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2],
#     [2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2],
#     [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 2],
#     [2, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
# ])
#
# map12_spawn = np.array([
#     [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
#     [2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2],
#     [2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2],
#     [2, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 2],
#     [2, 3, 3, 3, 3, 1, 0, 0, 0, 0, 4, 4, 4, 4, 2],
#     [2, 3, 3, 3, 3, 1, 0, 0, 0, 0, 4, 4, 4, 4, 2],
#     [2, 3, 3, 3, 3, 1, 0, 0, 0, 0, 4, 4, 4, 4, 2],
#     [2, 3, 3, 3, 3, 1, 0, 0, 0, 1, 4, 4, 4, 4, 2],
#     [2, 3, 3, 3, 3, 1, 0, 0, 0, 1, 1, 1, 0, 0, 2],
#     [2, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 2],
#     [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
# ])

exit_test_map = [(10, 3), (10, 7)]
start_points_test_map = [(4, 7),(4.5, 1),(2.125, 7),(4.5, 7),(6.0125, 7)]
radius_test_map = 0
test_map = np.array([
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 1, 1, 1, 1, 0, 0, 3],
    [2, 0, 0, 0, 0, 1, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 3],
    [2, 0, 0, 0, 0, 0, 0, 0, 0, 2],
    [2, 2, 2, 2, 2, 2, 2, 2, 2, 2],
])

map0111 = np.loadtxt("E:/论文/PedestrainSimulation -test-/ped_env/data/data.txt", dtype=np.int32, encoding='UTF-8',delimiter=',')
#print(map0111)
map0111_spawn=np.loadtxt("E:/论文/PedestrainSimulation -test-/ped_env/data/data1.txt", dtype=np.int32, encoding='UTF-8',delimiter=',')
#print(map0111_spawn)
start_points_map0111 = [(1, 1),(1, 1)]
exit_map0111 = [(6.5,39.5),(6.5,0.5)]
random_exits0111 = [[3], [4]]
radius_map0111=1
map_0111 = Map(map0111, exit_map0111,start_points_map0111,random_exits0111,"map_0111",radius_map0111,map0111_spawn)
#map_01 =  Map(map1, exit_map1, start_points_map1, random_exits1, "map_01", radius_map1)
map_02 = Map(map2, exit_map2, start_points_map2, random_exits2, "map_02", radius_map2, map2_spawn)
map_05 = Map(map5, exit_map5, start_points_map5, random_exits5, "map_05", radius_map5, map5_spawn) #simple 1600 16 51.01min TD3
map_06 = Map(map6, exit_map6, start_points_map6, random_exits6, "map_06", radius_map6, map6_spawn) #hard 1600 16 6_map11_use hour TD3
#map_07 =  Map(map7, exit_map7, start_points_map7, random_exits7, "map_07", radius_map7)
#!map_08 = Map(map8, exit_map8, start_points_map8, random_exits8, "map_08", radius_map8) #
# map_09 =  Map(map9, exit_map9, start_points_map9, random_exits9, "map_09", radius_map9)
map_10 =  Map(map10, exit_map10, start_points_map10, random_exits10, "map_10", radius_map10, map10_spawn)
map_11 =  Map(map11, exit_map11, start_points_map11, random_exits11, "map_11", radius_map11, map11_spawn)
map_12 =  Map(map12, exit_map12, start_points_map12, random_exits12, "map_12", radius_map12, map12_spawn)
#map_test = Map(test_map, exit_test_map, start_points_test_map, "map_test", radius_test_map)
print(map_0111.map)