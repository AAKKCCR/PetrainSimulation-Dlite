import pygame
import random
import math

# 初始化 Pygame
pygame.init()

# 设置窗口参数
window_width = 800
window_height = 600
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Social Force Simulation")

# 参数设置
num_agents = 100  # 行人数量
speed = 1.0  # 行人的基本速度
radius = 10.0  # 行人的半径
alpha = 200.0  # Alpha参数
beta = 0.08  # Beta参数

# 创建行人
agents = []
for i in range(num_agents):
    x = random.uniform(50, window_width - 50)
    y = random.uniform(50, window_height - 50)
    vx = random.uniform(-speed, speed)
    vy = random.uniform(-speed, speed)
    agents.append([x, y, vx, vy])

# 更新函数
def update_agents():
    for i in range(num_agents):
        x, y, vx, vy = agents[i]

        # 计算每个行人的位移
        preferred_velocity = [0.0, 0.0]

        # 计算社会力
        social_force = [0.0, 0.0]
        for j in range(num_agents):
            if i != j:
                dx = agents[j][0] - x
                dy = agents[j][1] - y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance < 2 * radius:
                    direction = [dx / distance, dy / distance]
                    social_force[0] += alpha * math.exp((2 * radius - distance) / beta) * direction[0]
                    social_force[1] += alpha * math.exp((2 * radius - distance) / beta) * direction[1]

        # 计算行人的位移
        desired_velocity = [speed * preferred_velocity[0] + social_force[0], speed * preferred_velocity[1] + social_force[1]]
        x += desired_velocity[0]
        y += desired_velocity[1]
        agents[i] = [x, y, vx, vy]

# 主循环
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 更新行人位置
    update_agents()

    # 清空窗口
    window.fill((255, 255, 255))

    # 绘制行人
    for agent in agents:
        x, y, _, _ = agent
        pygame.draw.circle(window, (255, 0, 0), int(x), int(y), 10)

    # 更新显示
    pygame.display.flip()

    # 控制帧速率
    clock.tick(60)

# 退出 Pygame
pygame.quit()
