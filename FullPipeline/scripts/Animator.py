from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.common.file_reader import CommonRoadFileReader
import numpy as np

# scenario filename
filename = "Collision-Analysis-Model/FullPipeline/output/OUT0.xml"

scenario_path =  filename # Replace with your scenario file path
scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

plan = []
for planning_problem in planning_problem_set.planning_problem_dict.values():
    plan.append(planning_problem)



config= ReactivePlannerConfiguration()

# Update configurations with the scenario and planning problems
config.update(scenario=scenario, planning_problem=plan[0])
# run route planner to compute route

route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)
route = route_planner.plan_routes()
route = route[0]
reference_path = []

for i in route.lanelet_ids:
    lan = scenario.lanelet_network.find_lanelet_by_id(i)
    reference_path.extend(lan.center_vertices)
ctv = []
i = 0
for pt in reference_path:
    i+=1
    pt = [pt[0],pt[1]]
    if i%5 == 0:
        ctv.append(pt)




print(route)

# initialize reactive planner
planner = ReactivePlanner(config)
print(ctv, len(reference_path))
reference_path = np.array(ctv)

# set reference path and desired velocity (here we just keep the current speed)
planner.set_reference_path(reference_path)
planner.set_desired_velocity(current_speed=planner.x_0.velocity)

# call plan function
optimal_result = planner.plan()