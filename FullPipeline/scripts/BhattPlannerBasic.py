from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.common.util import Interval, AngleInterval
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.lanelet import Lanelet
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep
from scipy.spatial import KDTree
from commonroad.common.file_writer import CommonRoadFileWriter
import math
from shapely.geometry import Point
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
import numpy as np
from commonroad.scenario.trajectory import Trajectory
from scipy.interpolate import splprep, splev
from commonroad.scenario.state import InitialState, KSState

steps = 40

def dist(a,b):
    if (a[0] == b[0]) or (a[1] == b[1]):
        return 0
    else:
        return 1

def find_traj(scenario,planprob):


    config = ReactivePlannerConfiguration()

    config.update(scenario=scenario, planning_problem=planprob)

    route_planner = RoutePlanner(config.scenario.lanelet_network, config.planning_problem)

    router = route_planner.plan_routes()
    
    
    route = router[0]
    route = route.lanelet_ids
    print(route)

    center_points = []


    for i in route:
        lan = scenario.lanelet_network.find_lanelet_by_id(i)
        ctv = lan.center_vertices
        
        for pt in ctv:
            pt = [pt[0],pt[1]]
            if len(center_points) == 0 or dist(pt,center_points[len(center_points)-1])==1:
                center_points.append(pt)

            





    # for i in route:
    #     lan = scenario.lanelet_network.find_lanelet_by_id(i)
    #     german =[lan.center_vertices[0]]
    #     j=0
    #     for k in lan.center_vertices:
    #         j+=1
    #         print(k,"kk",german[len(german)-1])
    #         if dist(german[len(german)-1],k)==0 or j%10!=0:
    #             print(k,german,"GERMAN")
    #         else:
    #             german.append(k)

    #     center_points.extend(german)





# Initial state for vehicle 2 (Kittredge St, traveling east)
    lst=[]
    for i in range(len(center_points)):
        lst.append([center_points[i][0],center_points[i][1]])




# Additional intermediate control points for Car 1 and Car 2
    # lst = np.array(lst)
    control_points_p1 = np.array(center_points)

    # control_points_p1 = [[1,2],[1,30],[1,40]]
    # control_points_p1 = np.array([[-26.8,24.2], [-26.8,24.4],[-26.4,6.4],[-26.4,6.1]])
    # print(control_points_p1)
    


# Create a parameterized spline for both trajectories
    tck_p1, u_p1 = splprep([control_points_p1[:, 0], control_points_p1[:, 1]], s=1e-3)


# Generate new points along the spline by evaluating it at regular intervals
    u_new = np.linspace(0, 1, steps)  # `steps` is the number of time steps you want

# Evaluate spline to get trajectory positions for both p1 and p2
    x_p1, y_p1 = splev(u_new, tck_p1)


# Now construct the KSState objects using the interpolated points
    dx_p1, dy_p1 = splev(u_new, tck_p1, der=1)


# Compute orientation as the arctangent of the gradient (dy/dx)
    orientation_p1 = np.arctan2(dy_p1, dx_p1)


# Construct the KSState objects using the interpolated points and orientations
    trajectory_states_p1 = [
        KSState(position=np.array([x_p1[i], y_p1[i]]), velocity=10, orientation=orientation_p1[i], time_step=i)
        for i in range(steps)
    ]
 
    # Create trajectories for both vehicles
    trajectory_p1 = Trajectory(initial_time_step=0, state_list=trajectory_states_p1)

    return trajectory_p1

