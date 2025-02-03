import os
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, KSState
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from matplotlib.animation import FuncAnimation
from commonroad.visualization.draw_params import TrafficLightParams
from commonroad.visualization.draw_params import TrafficLightParams, DynamicObstacleParams

# Set the scale factor to enlarge the traffic light
  # Increase to desired size


# Load the scenario using CommonRoadFileReader
scenario_file = "Commonroad/shifted_file.xml"
scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
def dist(point1, point2):
    return np.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)
# This is fine here but in general it is best we calcualte the path between 2 points and the distance of that path rather then a straght line since in tis exmapolelk opuir path is a straight line but in real life it is not.


# Define the vehicles' parameters
vehicle_shape = Rectangle(length=4.5, width=2.0)
vehicle_shape2 = Rectangle(length=8.5, width=3.0)  # Typical passenger car dimensions
collisionvertex1 = {'x': -1.3, 'y': 0.5}
collisionvertex2 = {'x': -4.0, 'y': -8.0}
A = {'x': 7.2, 'y':27.7}
B = {'x': -32.3, 'y': -9.4}
C = {'x': 14.0, 'y':0.4}
steps = 40
speed = steps/4

# Initial state for vehicle 1 (Shattuck Ave, traveling south)
initial_state_p1 = InitialState(
    position=np.array([A['x'],A['y']]),  
    velocity=dist(A,collisionvertex2)/(steps -1) ,                 
    orientation=-np.pi / 2,    
    time_step=0 
)

# Initial state for vehicle 2 (Kittredge St, traveling east)
initial_state_p2 = InitialState(
    position=np.array([B['x'], B['y']]),  
    velocity= dist(B,collisionvertex2)/(steps -1) ,                 
    orientation=0.0,            
    time_step=0
)

initial_state_p3 = InitialState(
    position=np.array([C['x'], C['y']]),
    velocity= dist(C,collisionvertex1)/(steps -1) ,                 
    orientation=0.0,            
    time_step=0
)



# Define the vehicles' trajectories

import numpy as np
from scipy.interpolate import splprep, splev

# Additional intermediate control points for Car 1 and Car 2
control_points_p1 = np.array([[A['x'], A['y']], [3.6,15.8],[-0.8,1.9],[-2.5,-3.6],[collisionvertex2['x']+1, collisionvertex2['y']+ 3]])  # Car 1
control_points_p2 = np.array([[B['x'], B['y']], [-18.2,-7.1],[-9.5,-6.2] ,[collisionvertex2['x'], collisionvertex2['y']]])  # Car 2
control_points_p3 = np.array([[C['x'], C['y']], [8.8,-0.4],[1,-2.1] ,[collisionvertex1['x'] +1, collisionvertex1['y'] - 1]])  # Car 2

# Create a parameterized spline for both trajectories
tck_p1, u_p1 = splprep([control_points_p1[:, 0], control_points_p1[:, 1]], s=0)
tck_p2, u_p2 = splprep([control_points_p2[:, 0], control_points_p2[:, 1]], s=0)
tck_p3, u_p3 = splprep([control_points_p3[:, 0], control_points_p3[:, 1]], s=0)
# Generate new points along the spline by evaluating it at regular intervals
u_new = np.linspace(0, 1, steps)  # `steps` is the number of time steps you want

# Evaluate spline to get trajectory positions for both p1 and p2
x_p1, y_p1 = splev(u_new, tck_p1)
x_p2, y_p2 = splev(u_new, tck_p2)
x_p3, y_p3 = splev(u_new, tck_p3)
# Now construct the KSState objects using the interpolated points
dx_p1, dy_p1 = splev(u_new, tck_p1, der=1)
dx_p2, dy_p2 = splev(u_new, tck_p2, der=1)
dx_p3, dy_p3 = splev(u_new, tck_p3, der=1)

# Compute orientation as the arctangent of the gradient (dy/dx)
orientation_p1 = np.arctan2(dy_p1, dx_p1)
orientation_p2 = np.arctan2(dy_p2, dx_p2)
orientation_p3 = np.arctan2(dy_p3, dx_p3)
# Construct the KSState objects using the interpolated points and orientations
trajectory_states_p1 = [
    KSState(position=np.array([x_p1[i], y_p1[i]]), velocity=10, orientation=orientation_p1[i], time_step=i)
    for i in range(steps)
]

trajectory_states_p2 = [
    KSState(position=np.array([x_p2[i], y_p2[i]]), velocity=10, orientation=orientation_p2[i], time_step=i)
    for i in range(steps)
]

trajectory_states_p3 = [
    KSState(position=np.array([x_p3[i], y_p3[i]]), velocity=10, orientation=orientation_p3[i], time_step=i)
    for i in range(steps)
]


# Create trajectories for both vehicles
trajectory_p1 = Trajectory(initial_time_step=0, state_list=trajectory_states_p1)
trajectory_p2 = Trajectory(initial_time_step=0, state_list=trajectory_states_p2)
trajectory_p3 = Trajectory(initial_time_step=0, state_list=trajectory_states_p3)

# Create a TrajectoryPrediction object for both vehicles
prediction_p1 = TrajectoryPrediction(trajectory=trajectory_p1, shape=vehicle_shape2)
prediction_p2 = TrajectoryPrediction(trajectory=trajectory_p2, shape=vehicle_shape)
prediction_p3 = TrajectoryPrediction(trajectory=trajectory_p3, shape=vehicle_shape)

car1_draw_params = DynamicObstacleParams()
car1_draw_params.draw_obstacle_color = "#FF5733"  # Custom color for car1 (e.g., orange)
#Define the vehicles as dynamic obstacles with predictions
car1 = DynamicObstacle(
    obstacle_id=1000,
    obstacle_type=ObstacleType.CAR,
    obstacle_shape=vehicle_shape2,
    initial_state=initial_state_p1,
    prediction=prediction_p1
)

car2 = DynamicObstacle(
    obstacle_id=2000,
    obstacle_type=ObstacleType.CAR,
    obstacle_shape=vehicle_shape,
    initial_state=initial_state_p2,
    prediction=prediction_p2
)

car3 = DynamicObstacle(
    obstacle_id=3000,
    obstacle_type=ObstacleType.CAR,
    obstacle_shape=vehicle_shape,
    initial_state=initial_state_p3,
    prediction=prediction_p3
)


# # Add the vehicles to the scenario
scenario.add_objects(car1)
scenario.add_objects(car2)
scenario.add_objects(car3)

# Create animation of the scenario (moving vehicles)
fig, ax = plt.subplots(figsize=(10, 10))
rnd = MPRenderer()
# Assuming TrafficLightParams is already defined in your project
custom_traffic_light_params = TrafficLightParams(
    scale_factor=8.2,                  
)


def animate(time_step):
    ax.clear()  # Clear the plot for each frame
    rnd = MPRenderer()
    rnd.draw_params.traffic_light = custom_traffic_light_params  # Apply custom traffic light params
    rnd.draw_params.time_begin = time_step    
    scenario.draw(rnd)
    rnd.render()
    ax.set_title(f"Time Step: {time_step}")
    ax.set_xlim([-50, 50])
    ax.set_ylim([-50, 50])


ani = FuncAnimation(fig, animate, frames=len(trajectory_states_p1), interval = (1/speed)*300, repeat=False)

# Display the animation
plt.show()
