import os
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, KSState
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from matplotlib.animation import FuncAnimation
from commonroad.visualization.draw_params import TrafficLightParams

# Set the scale factor to enlarge the traffic light
traffic_light_params = TrafficLightParams(scale_factor=5.0)  # Increase to desired size


# Load the scenario using CommonRoadFileReader
scenario_file = "Commonroad/shifted_file2.xml"
scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
def dist(point1, point2):
    return np.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)
# This is fine here but in general it is best we calcualte the path between 2 points and the distance of that path rather then a straght line since in tis exmapolelk opuir path is a straight line but in real life it is not.


# Define the vehicles' parameters
vehicle_shape = Rectangle(length=4.5, width=2.0)
vehicle_shape2 = Rectangle(length=3.5, width=0.5)  # Typical passenger car dimensions
collisionvertex = {'x': 6.4, 'y': -6.0}
A = {'x': 8.2, 'y': -37}
B = {'x': -1.5, 'y': 32.6}
steps = 40
speed = steps/4

# Initial state for vehicle 1 (Shattuck Ave, traveling south)
initial_state_p1 = InitialState(
    position=np.array([A['x'],A['y']]),  
    velocity=dist(A,collisionvertex)/(steps -1) ,                 
    orientation=-np.pi / 2,    
    time_step=0 
)

# Initial state for vehicle 2 (Kittredge St, traveling east)
initial_state_p2 = InitialState(
    position=np.array([B['x'], B['y']]),  
    velocity= dist(B,collisionvertex)/(steps -1) ,                 
    orientation=0.0,            
    time_step=0
)



# Define the vehicles' trajectories

import numpy as np
from scipy.interpolate import splprep, splev

# Additional intermediate control points for Car 1 and Car 2
control_points_p1 = np.array([[A['x'], A['y']],[6.4,-16.9], [6.3,-9.9],[collisionvertex['x'], collisionvertex['y']-2]])  # Car 1
control_points_p2 = np.array([[B['x'], B['y']],[0.2,15.7],[0.8,9.7],[1.8, -0.8],[3.8,-4.8] ,[collisionvertex['x'], collisionvertex['y']]])  # Car 2


# Create a parameterized spline for both trajectories
tck_p1, u_p1 = splprep([control_points_p1[:, 0], control_points_p1[:, 1]], s=0)
tck_p2, u_p2 = splprep([control_points_p2[:, 0], control_points_p2[:, 1]], s=0)

# Generate new points along the spline by evaluating it at regular intervals
u_new = np.linspace(0, 1, steps)  # `steps` is the number of time steps you want

# Evaluate spline to get trajectory positions for both p1 and p2
x_p1, y_p1 = splev(u_new, tck_p1)
x_p2, y_p2 = splev(u_new, tck_p2)

# Now construct the KSState objects using the interpolated points
dx_p1, dy_p1 = splev(u_new, tck_p1, der=1)
dx_p2, dy_p2 = splev(u_new, tck_p2, der=1)

# Compute orientation as the arctangent of the gradient (dy/dx)
orientation_p1 = np.arctan2(dy_p1, dx_p1)
orientation_p2 = np.arctan2(dy_p2, dx_p2)

# Construct the KSState objects using the interpolated points and orientations
trajectory_states_p1 = [
    KSState(position=np.array([x_p1[i], y_p1[i]]), velocity=10, orientation=orientation_p1[i], time_step=i)
    for i in range(steps)
]

trajectory_states_p2 = [
    KSState(position=np.array([x_p2[i], y_p2[i]]), velocity=10, orientation=orientation_p2[i], time_step=i)
    for i in range(steps)
]


# Create trajectories for both vehicles
trajectory_p1 = Trajectory(initial_time_step=0, state_list=trajectory_states_p1)
trajectory_p2 = Trajectory(initial_time_step=0, state_list=trajectory_states_p2)

# Create a TrajectoryPrediction object for both vehicles
prediction_p1 = TrajectoryPrediction(trajectory=trajectory_p1, shape=vehicle_shape2)
prediction_p2 = TrajectoryPrediction(trajectory=trajectory_p2, shape=vehicle_shape)

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

# # Add the vehicles to the scenario
scenario.add_objects(car1)
scenario.add_objects(car2)

# Create animation of the scenario (moving vehicles)
fig, ax = plt.subplots(figsize=(10, 10))
rnd = MPRenderer()
custom_traffic_light_params = TrafficLightParams(
    scale_factor=2.2,                  # Custom scale factor
)



def animate(time_step):
    ax.clear()  # Clear the plot for each frame
    rnd = MPRenderer()
    rnd.draw_params.traffic_light = custom_traffic_light_params  # Set the custom scale factor for traffic lights
    rnd.draw_params.time_begin = time_step
    scenario.draw(rnd)
    rnd.render()
    ax.set_title(f"Time Step: {time_step}")
    ax.set_xlim([-50, 50])
    ax.set_ylim([-50, 50])


ani = FuncAnimation(fig, animate, frames=len(trajectory_states_p1), interval = (1/speed)*300, repeat=False)
# Display the animation
plt.show()

