from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.common.file_reader import CommonRoadFileReader
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.scenario.state import KSState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle
from commonroad.common.util import Interval, AngleInterval
from commonroad.prediction.prediction import TrajectoryPrediction



filename = "Collision-Analysis-Model/FullPipeline/output/OUT0.xml"
scenario_path =  filename 
scenario, planning_problem_set= CommonRoadFileReader(scenario_path).open()
print(planning_problem_set)



plan = []

for planning_problem in planning_problem_set.planning_problem_dict.values():
    plan.append(planning_problem)
 


for i in range(len(plan)):
    print(len(plan),"43e33343")

    config = ReactivePlannerConfiguration()


    config.update(scenario=scenario, planning_problem=plan[i])

    route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)

    router = route_planner.plan_routes()
    
    route = router[0]
    
    print(route.lanelet_ids)

    planner = ReactivePlanner(config)


    planner.set_reference_path(route.reference_path)
    planner.set_desired_velocity(current_speed=planner.x_0.velocity)

    # call plan function
    optimal_result = planner.plan()
    if optimal_result == None:
        print(i, "yayayayay")
        
    else:
    # print(optimal_result)
    # print(plan[0].initial_state)



        trajectory_p1 = optimal_result[0]
        vehicle_shape = Rectangle(length=4.5, width=2.0)

        prediction_p1 = TrajectoryPrediction(trajectory=trajectory_p1, shape=vehicle_shape)

        car1 = DynamicObstacle(
            obstacle_id=scenario.generate_object_id(),
            obstacle_type=ObstacleType.CAR,
            obstacle_shape=vehicle_shape,
            initial_state=plan[i].initial_state,
            prediction=prediction_p1
        )

        scenario.add_objects(car1)



fig, ax = plt.subplots(figsize=(10, 10))
rnd = MPRenderer()
# custom_traffic_light_params = TrafficLightParams(
#     scale_factor=2.2,                  # Custom scale factor
# )



def animate(time_step):
    ax.clear()  # Clear the plot for each frame
    rnd = MPRenderer()
    rnd.draw_params.time_begin = time_step
    scenario.draw(rnd)
    rnd.render()
    ax.set_title(f"Time Step: {time_step}")
    ax.set_xlim([-50, 50])
    ax.set_ylim([-50, 50])


ani = FuncAnimation(fig, animate, frames=300, interval = (1/20)*300, repeat=False)
# Display the animation
plt.show()