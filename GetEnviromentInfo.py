# -- coding:UTF-8 --
# !/usr/bin/env python
# Author:Kin Zhang
# email: kin_eng@163.com

import glob
import os
import sys
import time
import math
import weakref
import csv
import pygame
import numpy as np
import datetime
import matplotlib.pyplot as plt
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class People:
    def __init__(self, pedestrian):
        self.p = pedestrian  # carla中的行人
        self.m = 50 + random.randint(0, 20)  # 行人质量/kg
        self.r = (35 + random.randint(0, 5)) / 2  # 行人半径(肩宽/2)/cm
        self.d_v = (20 + random.randint(0, 20)) / 100

# 确定XY坐标
def show_point(world, point_location):
    world.debug.draw_string(point_location, 'P', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100000,
                            persistent_lines=True)


def show_x_point(world, point_location):
    world.debug.draw_string(point_location, 'X', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100000,
                            persistent_lines=True)


def show_y_point(world, point_location):
    world.debug.draw_string(point_location, 'Y', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100000,
                            persistent_lines=True)


def show_O_point(world, point_location):
    world.debug.draw_string(point_location, 'O', draw_shadow=False,
                            color=carla.Color(r=0, g=255, b=0), life_time=100000,
                            persistent_lines=True)

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

class ObstacleSensor(object):
    def __init__(self, parent_actor, world):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        bp = world.get_blueprint_library().find('sensor.other.obstacle')
        bp.set_attribute('distance', str(0))
        bp.set_attribute('hit_radius', str(0.7))
        bp.set_attribute('debug_linetrace', 'true')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=parent_actor)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: ObstacleSensor._on_detect(weak_self, event))

    # def get_detection_history(self):
    #
    #     for frame, intensity in self.history:
    #         history[frame] += intensity
    #     return history
    #
    # @staticmethod
    def _on_detect(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        actor_self = get_actor_display_name(event.actor)
        print('\nDetectObstacle with %r' % actor_type)
        print('Self:%r' % actor_self)
        # impulse = event.normal_impulse
        # intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        # self.history.append((event.frame, intensity))
        # if len(self.history) > 4000:
        #     self.history.pop(0)




def DrawBoundingBoxes(world, location, distance, tags):
    for tag in tags:
        env_objs = world.get_level_bbs(tag)
        for building in env_objs:
            if building.location.distance(location) < distance:
                print(building)
                world.debug.draw_box(building, building.rotation, 0.05, carla.Color(255, 0, 0, 0), 0)


# class carla_matrix(object):
#     def __int__(self,):

class AstarMap(object):
    #XYchanges +1或者-1 表示左上角相对xy轴方向,width,height表示矩阵的长宽
    def __init__(self, world, origin, width, height, ScaleX, ScaleY, Xchanges=1, Ychanges=1):
        self.world = world
        self.origin = origin
        self.width = width
        self.height = height
        self.ScaleX = ScaleX
        self.ScaleY = ScaleY
        self.walker_bp = world.get_blueprint_library().filter('walker.pedestrian.0001')
        self.Xchanges = Xchanges
        self.Ychanges = Ychanges
        self.blueprint_library = world.get_blueprint_library()
    #矩阵中的点转化为实际世界坐标,x,y表示的是矩阵中的位置，第几行第几列
    def point_to_location(self, x, y):
        if self.Xchanges == self.Ychanges:
            position_x = self.Xchanges * (y - 1) * self.ScaleX + 1 / 2 * self.ScaleX + self.origin.x
            position_y = self.Ychanges * (x - 1) * self.ScaleY + 1 / 2 * self.ScaleY + self.origin.y
        else:
            position_x = self.Xchanges * (x - 1) * self.ScaleX + 1 / 2 * self.ScaleX + self.origin.x
            position_y = self.Ychanges * (y - 1) * self.ScaleY + 1 / 2 * self.ScaleY + self.origin.y
        return position_x, position_y

    #实际坐标转化为矩阵中的点
    def location_to_point(self, position_x, position_y):
        X = (position_x - self.origin.x)//self.ScaleX #方格内所有点都用中心点代替
        Y = (position_y - self.origin.y)//self.ScaleY
        return X, Y

    #获取地图矩阵
    def get_barrier_list(self):
        # A = np.array([[4, 3, 2, 4], [5, 4, 7, 8], [9, 16, 11, 5], [13, 3, 4, 16], [6, 18, 1, 20]])
        self.With_num = math.ceil((self.width / self.ScaleX)) #方格数量向上取整
        self.Height_num = math.ceil((self.height / self.ScaleY))
        print(self.ScaleX)
        print(self.ScaleY)
        print(self.width)
        print(self.height)
        print(self.With_num)
        print(self.Height_num)
        barrier_numpy = [[]for x in range(self.Height_num)]
        print(barrier_numpy)
        for i in range(0, self.Height_num):
            for j in range(0, self.With_num):
                X, Y = self.point_to_location(i, j)
                print(X,Y)
                spawn_point = carla.Transform(carla.Location(X, Y, 2))
                bp = self.blueprint_library.filter('static.prop.atm')
                walker_bp = random.choice(bp)
                player = self.world.try_spawn_actor(walker_bp, spawn_point)
                # player = self.world.try_spawn_actor(self.walker_bp, spawn_point)
                if i is 0 or i == self.Height_num-1:
                    barrier_numpy[i].append(2)
                elif j is 0 or j == self.With_num-1:
                    barrier_numpy[i].append(2)
                elif player is None:
                    print("wrong")
                    barrier_numpy[i].append(1)
                else:
                    barrier_numpy[i].append(0)
                    # print(spawn_point)
                    player.destroy()
                    player = None
                if player is not None:
                    player.destroy()
                    player = None

        print(barrier_numpy)
        np.savetxt("./test.txt", barrier_numpy,fmt="%d",delimiter=',')
        ax=plt.matshow(barrier_numpy, cmap=plt.cm.Reds)
        plt.title("barrides")
        plt.colorbar(ax.colorbar, fraction=0.025)
        plt.show()
        # barrier_list =np.array()
        # for i in range(0,3):
        #     for j in range(0,4):
        #         barrier_list[i][j] = 1

    #def next_loc(self, x , y):
def SetNumpy(col,row):
    barrier_numpy = [[] for x in range(row)]
    for i in range(0, row):
        for j in range(0, col):
            if i is 0 or j is 0 or i == row-1 or j == col-1:
                barrier_numpy[i].append(2)
            else:
                barrier_numpy[i].append(0)
    print(barrier_numpy)
    np.savetxt("./test.txt", barrier_numpy, fmt="%d", delimiter=',')
    ax = plt.matshow(barrier_numpy, cmap=plt.cm.Reds)
    plt.title("barrides")
    plt.colorbar(ax.colorbar, fraction=0.025)
    plt.show()


def Convert_to_carla(origin ,npX ,npY, scaleX, scaleY , xChanges, yChanges):
    carlaX = (float(npY) / scaleX) * xChanges + origin.x
    carlaY = (float(npX) / scaleY) * yChanges + origin.y
    return carlaX, carlaY

def ped_move(pedestrian,move_dir,loc,ped_speed):
	pedestrian.apply_control(carla.WalkerControl(move_dir,ped_speed,False))
	pedestrian.set_location(loc)
	#没有apply_control 人瞬移，没有set_location 人跟不上头上的编号，会慢


def main():
    global pedestrain
    synchronous_master = False
    Pedestrians = []
    Persons = {}
    try:
        SpawnActor = carla.command.SpawnActor
        client = carla.Client(host='127.0.0.1', port=2000)
        client.set_timeout(60.0)
        world = client.get_world()


        #world = client.load_world("Town01")
        # 切换地图
        print(world.cast_ray(carla.Location(10, 0, 0), carla.Location(229, 116, 2)))
        print("list")
        # pygame.init()
        # pygame.font.init()
        # display = pygame.display.set_mode((500, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)  # 窗口可以自己修改大小
        # pygame.display.set_caption("Test Vision")

        start_point = carla.Location(229, 116, 2)
        end_point = carla.Location(20, 194, 2)
        origin_point = carla.Location(0, 0, 0)
        x_axis = carla.Location(10, 0, 0)
        y_axis = carla.Location(0, 10, 0)
        show_point(world, start_point)
        show_point(world, end_point)
        show_O_point(world, origin_point)
        show_x_point(world, x_axis)
        show_y_point(world, y_axis)
        # 显示坐标原点和坐标轴
        # world=client.get_world()

        # 拿到这个世界所有物体的蓝图
        blueprint_library = world.get_blueprint_library()


        # 随机选择人物形象

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera = world.spawn_actor(
            blueprint=camera_bp,
            transform=carla.Transform(carla.Location(x=240, y=110, z=20)))
        print(world.get_weather())
        i = 248
        # 测试生成

        world=client.get_world()

        # #绘制地图
        Map = AstarMap(world, Node(-3, -103), 20, 20, 0.5, 0.5, 1, 1)
        Map.get_barrier_list()

        # Map = AstarMap(world, Node(51, 124), 4, 7, 1, 1, -1, -1)
        # Map.get_barrier_list()



        #放置观察者
        settings = world.get_settings()
        # settings.fixed_delta_seconds = 0.05  # 20 fps, 5ms
        world.apply_settings(settings)
        spectator = world.get_spectator()



        #添加的静态网格体的tag为NONE
        tags = []
        tags.append(carla.CityObjectLabel.Walls)
        tags.append(carla.CityObjectLabel.Buildings)
        tags.append(carla.CityObjectLabel.Static)
        tags.append(carla.CityObjectLabel.NONE)
        tags.append(carla.CityObjectLabel.Fences)
        tags.append(carla.CityObjectLabel.TrafficLight)
        tags.append(carla.CityObjectLabel.Other)
        tags.append(carla.CityObjectLabel.GuardRail)
        tags.append(carla.CityObjectLabel.Ground)
        tags.append(carla.CityObjectLabel.Roads)
        tags.append(carla.CityObjectLabel.Sidewalks)



        bp = blueprint_library.filter('walker.pedestrian.*')



    finally:
        print('\ndestroying pedestrain')
        for pedestrain in Pedestrians:
            pedestrain.p.destroy()
        for ped in Persons.values():
            if ped is None:
                continue
            ped.destroy()



if __name__ == '__main__':

    try:
        SetNumpy(20 , 50)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
