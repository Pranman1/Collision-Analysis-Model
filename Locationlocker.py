from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.common.util import Interval
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.visualization.mp_renderer import MPRenderer
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep
from scipy.spatial import KDTree





vehicle_shape = Rectangle(length=4.5, width=2.0)  
collisionvertex = {'x': -24.3, 'y': -2.7}
A = {'x': -27.1, 'y': 30}
B = {'x': -47.7, 'y': -7}
steps = 40
speed = steps/4



# First we'll pqLoad Lanelet Network from XML
scenario_file =  "Commonroad/output/map.xml"
scenario,_= CommonRoadFileReader(scenario_file).open()


# Extract all lanelet center points
center_points = []
num = {}
for lanelet in scenario.lanelet_network.lanelets:
    a = len(lanelet.center_vertices)//2
    num[f"{lanelet.lanelet_id}"] = lanelet.center_vertices[a]
    center_points.extend(lanelet.center_vertices)

# Convert to numpy array for easier processing
center_points = np.array(center_points)

# Use KDTree to find the densest region
kdtree = KDTree(center_points)

# Define a search radius (adjust as needed)
search_radius = 5.0

# Count the number of points in the neighborhood of each center point
point_density = np.array([len(kdtree.query_ball_point(point, search_radius)) for point in center_points])

# Find the index of the densest point
densest_point_index = np.argmax(point_density)

# Get the coordinate of the densest area
densest_point = center_points[densest_point_index]

print("Densest point coordinate:", densest_point)
print("Number of lanelet center points in the area:", point_density[densest_point_index])

scenario.translate_rotate(-densest_point,0)

for lanelet in scenario.lanelet_network.lanelets:
    if num[f"{lanelet.lanelet_id}"][0]>3:
        scenario.lanelet_network.remove_lanelet(lanelet.lanelet_id)

#Define Two Vehicles with Starting Points and Goal Regions
planning_problem_set = PlanningProblemSet()

# Vehicle 1
start_state_1 = InitialState(
    time_step=0,
    position=np.array([-27.1, 30.0]),
    velocity=5.0,
    orientation=0.0,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=0.0  # Slip angle
)
goal_state_1 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(4.0, 6.0)
)
goal_region_1 = GoalRegion([goal_state_1])
planning_problem_1 = PlanningProblem(1, initial_state=start_state_1, goal_region=goal_region_1)

# Vehicle 2
start_state_2 = InitialState(
    time_step=0,
    position=np.array([-47.7, -7.0]),
    velocity=7.0,
    orientation=0.0,  
    yaw_rate=0.0,  
    slip_angle=0.0 
)
goal_state_2 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(6.0, 8.0)
)
goal_region_2 = GoalRegion([goal_state_2])
planning_problem_2 = PlanningProblem(2, initial_state=start_state_2, goal_region=goal_region_2)


planning_problem_set.add_planning_problem(planning_problem_1)
planning_problem_set.add_planning_problem(planning_problem_2)




# Some problems here: The centerpoints dont give us equal steps wwe will have to use linspace
#  to add npoints based on our step size and sitace which is another layer of code. 
# We also have no way to add these objects as of yet, for this we will need to use the rective palnner which will be our next step

fig, ax = plt.subplots(figsize=(10, 10))
rnd = MPRenderer()
scenario.draw(rnd)
rnd.render()
ax.set_xlim([-50, 50])
ax.set_ylim([-50, 50])

plt.show()