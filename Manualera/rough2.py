from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.common.util import Interval, AngleInterval
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.visualization.mp_renderer import MPRenderer
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
from commonroad.prediction.prediction import TrajectoryPrediction
# scenario filename
filename = "Commonroad/USA_a1-1_1_T-1.xml"
scenario_path =  filename # Replace with your scenario file path
scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

num = {}
for lanelet in scenario.lanelet_network.lanelets:
    a = len(lanelet.center_vertices)//2
    num[f"{lanelet.lanelet_id}"] = lanelet.center_vertices[a]



for lanelet in scenario.lanelet_network.lanelets:
    if num[f"{lanelet.lanelet_id}"][0]<3:
        scenario.lanelet_network.remove_lanelet(lanelet.lanelet_id)




planning_problem_set = PlanningProblemSet()

# Vehicle 1
start_state_1 = InitialState(
    time_step=0,
    position=np.array([-28.4, 47.2]),
    velocity=5.0,
    orientation=-3.141/2,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=0.0,  # Slip angle
    acceleration=0.0
)
goal_state_1 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.1, -2.3])),
    velocity=Interval(4.0, 6.0)
)
goal_region_1 = GoalRegion([goal_state_1])
planning_problem_1 = PlanningProblem(1, initial_state=start_state_1, goal_region=goal_region_1)

# Vehicle 2
start_state_2 = InitialState(
    time_step=0,
    position=np.array([-47.5, -6.2]),
    velocity=7.0,
    orientation=0.0,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=0.0, # Slip angle
    acceleration=0.0
)
goal_state_2 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.1, -2.3])),
    velocity=Interval(6.0, 8.0)
)
goal_region_2 = GoalRegion([goal_state_2])
planning_problem_2 = PlanningProblem(2, initial_state=start_state_2, goal_region=goal_region_2)

# Add to Planning Problem Set
planning_problem_set.add_planning_problem(planning_problem_1)
planning_problem_set.add_planning_problem(planning_problem_2)


# Build planner configuration object (includes loading scenario)
plan = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
i = 0
for planning_problem in planning_problem_set.planning_problem_dict.values():
    plan[i] = planning_problem
    i+=1

# print(i)

config= ReactivePlannerConfiguration()

# Update configurations with the scenario and planning problems
config.update(scenario=scenario, planning_problem=plan[0])
# run route planner to compute route
route_planner = RoutePlanner(config.scenario, config.planning_problem)
route = route_planner.plan_routes().retrieve_first_route()
print(route.list_ids_lanelets)

# initialize reactive planner
planner = ReactivePlanner(config)
print(planner.x_0.velocity)
print(route.reference_path)
# set reference path and desired velocity (here we just keep the current speed)
planner.set_reference_path(route.reference_path)
planner.set_desired_velocity(current_speed=planner.x_0.velocity)
print(config)
# call plan function
optimal_result = planner.plan()

print(optimal_result)