import Box2D as b2d

from ped_env.envs import PedsMoveEnv as Env
from ped_env.pathfinder import AStarPolicy
from ped_env.utils.maps import *
from rl.utils.classes import make_parallel_env, PedsMoveInfoDataHandler


def HelloWorldProject():
    world = b2d.b2World()
    ground_body = world.CreateStaticBody(
        position=(0, -10),
        shapes=b2d.b2PolygonShape(box=(50, 10)),
    )
    body = world.CreateDynamicBody(position=(0, 4))

    box = body.CreatePolygonFixture(box=(1, 1),
                                    density=1,
                                    friction=0.3)
    timeStep = 1.0 / 60  # 时间步长，1/60秒
    vel_iters, pos_iters = 6, 2
    for i in range(600):  # 一共向前模拟60步，即总经过1秒
        world.Step(timeStep, vel_iters, pos_iters)

        # 清楚所有施加上的力，每次循环都是必须的
        world.ClearForces()

        # 打印输出物体的位置和角度
        print("Body Pos:{},Angle:{}".format(
            body.position,
            body.angle
        ))

def test2():
    import time
    import numpy as np

    debug = False
    save = True

    person_num = 40
    env = Env(map_12, person_num, group_size=(5, 5), frame_skipping=100, maxStep=250, debug_mode=debug, random_init_mode=True)
    leader_num = env.agent_count
    handler = PedsMoveInfoDataHandler(env.terrain, env.agent_count)

    for epoch in range(5):
        starttime = time.time()
        step = 0
        obs = env.reset()
        is_done = [False]
        while not is_done[0]:
            if not debug:
                action = np.random.random([leader_num, 9])
            else:
                action = np.zeros([leader_num, 9])
                action[:, 0] = 1
            obs, reward, is_done, info = env.step(action)
            handler.step(info)
            if debug:
                env.debug_step()
            step += env.frame_skipping
            env.render()
        handler.reset(info)
        endtime = time.time()
        print("智能体与智能体碰撞次数为{},与墙碰撞次数为{}!"
              .format(env.col_with_agent, env.col_with_wall))
        print("所有智能体在{}步后离开环境,离开用时为{},两者比值为{}!".format(step, endtime - starttime, step / (endtime - starttime)))
    handler.save("./")


def test3():
    import time
    import numpy as np

    debug = False
    # test1()
    person_num = 8
    n_rol_counts = 4
    total_epochs = 4
    _env = Env(map_05, person_num, group_size=(1, 1), maxStep=500, test_mode=debug)
    parallel_envs = make_parallel_env(_env, n_rol_counts)
    leader_num = parallel_envs.agent_count
    for epoch in range(total_epochs):
        starttime = time.time()
        step = 0
        obs = parallel_envs.reset()
        is_done = np.array([[False]])
        while not is_done[0, 0]:
            if not debug:
                action = np.random.random([n_rol_counts, leader_num, 9])
            else:
                action = np.zeros([n_rol_counts, leader_num, 9])
                action[:, :, 0] = 1
            obs, reward, is_done, info = parallel_envs.step(action)
            step += _env.frame_skipping
            # parallel_envs.render()
            # print(obs, reward, is_done)
        endtime = time.time()
        print("所有智能体在{}步后离开环境,离开用时为{},两者比值为{}!".format(step, endtime - starttime, step / (endtime - starttime)))

def test4():
    #使用连续动作空间的范例（随机策略）
    import time
    import numpy as np

    debug = False
    # test1()
    person_num = 40
    env = Env(map_0111, person_num, group_size=(5, 5), maxStep=10000, discrete=False, test_mode=debug)
    leader_num = env.agent_count
    # print(obs)
    for epoch in range(1):
        starttime = time.time()
        step = 0
        obs = env.reset()
        is_done = [False]
        while not is_done[0]:
            if not debug:
                action = (np.random.random([leader_num, 2]) - 0.5) * 2
            else:
                action = np.zeros([leader_num, 2])
                action[:, 0] = 1
            obs, reward, is_done, info = env.step(action)
            # print(obs[0][9:11])
            if debug:
                env.debug_step()
            step += env.frame_skipping
            env.render()
            # print(obs, reward, is_done)
        endtime = time.time()
        print("智能体与智能体碰撞次数为{},与墙碰撞次数为{}!"
              .format(env.col_with_agent, env.col_with_wall))
        print("所有智能体在{}步后离开环境,离开用时为{},两者比值为{}!".format(step, endtime - starttime, step / (endtime - starttime)))

def test5():
    import time

    debug = False

    person_num = 16
    env = Env(map_0111, person_num, group_size=(4, 4), frame_skipping=8, maxStep=2000, debug_mode=debug, random_init_mode=True)
    leader_num = env.agent_count
    policy = AStarPolicy(env)
    env.Policy = policy
    for epoch in range(1):
        starttime = time.time()
        step = 0
        # reset函数返回所有行人leader的当前位置，当前速度以及相对目标的位置
        obs = env.reset()
        policy.Init(env.leaders)
        is_done = [False]
        j = 0

        while not is_done[0]:
            action = policy.step(obs) #获得了所有leader的下一步的方向
            for i in range(1):
                obs, reward, is_done, info = env.step(action)
                env.render()
                if is_done[0]:
                    break
                if debug:
                    env.debug_step()
                # if j == 5:
                #     j = 0
                #     policy.setObstacle()
                step += env.frame_skipping
                j = j+1
        endtime = time.time()
        print("智能体与智能体碰撞次数为{},与墙碰撞次数为{}!"
              .format(env.col_with_agent, env.col_with_wall))
        print("所有智能体在{}步后离开环境,离开用时为{},两者比值为{}!".format(step, endtime - starttime, step / (endtime - starttime)))

if __name__ == '__main__':
    test5()

    # import kdtree
    # points = []
    # for i in range(100):
    #     points.append([10 - i * 0.5, 0.3 * i])
    # tr = kdtree.create(points, dimensions=2)
    # print(kdtree.visualize(tr))