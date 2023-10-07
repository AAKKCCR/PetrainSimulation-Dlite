from ped_env.run import test5
from carla_scenarioplay.scenarioplayback import replay_animation
def Simulation():
    ped_csvs = []
    veh_csvs = []
    veh_csvs.append("E:/论文/PedestrainSimulation/carla_scenarioplay/ped_data/veh.csv")
    path = "11"
    weather = 'Clear Noon'
    ped_csvs.append('E:/论文/PedestrainSimulation/carla_scenarioplay/ped_data/ped.csv')
    replay_animation(ped_csvs, veh_csvs, path, weather, no_render_mode=False, save_image=False)
if __name__ == '__main__':
    test5()
    Simulation()
