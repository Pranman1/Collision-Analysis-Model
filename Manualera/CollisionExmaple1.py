import matplotlib.pyplot as plt
import warnings
import numpy as np

# CommonRoad imports based on available modules
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.state import InitialState, KSState
from commonroad.geometry.shape import Rectangle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.prediction.prediction import TrajectoryPrediction
from matplotlib.animation import FuncAnimation



# Step 1: Create a new scenario
scenario = Scenario(dt = 0.0000001)  # Time step of 0.1 seconds

# veichle = [0,0,0,0,0,0,0,0,0,0,0]
# Veichles = input("Enter the number of vehicles: ")
# for i in range(int(Veichles)):
#     veichle[i] = input(f"Enter the party {i} type: 1 (car) 2 (bicycle) 3 (motorcycle) 4 (bus) 5 (truck) 6 (pedestrian) 7 (bicycle) 8 (motorcycle) 9 (bus) 10 (truck) ")

# For this example I have manually defined the lanelets but we can also use the commonroad library to import the lanelets from the OSM file. 


# I have hardcoded 8 lanelets for Shattuck (2 lanes southbound, 2 lanes northbound)
lanelet_shattuck_north_1 = Lanelet(
    left_vertices=np.array([[-3, 50],[-3, 0]]),
    right_vertices=np.array([[-5, 50],[-5, 0]]),
    center_vertices=np.array([[-4, 50], [-4, 0]]),
    lanelet_id=1000
)

lanelet_shattuck_north_2 = Lanelet(
    left_vertices=np.array([[3, 50], [3, 0]]),
    right_vertices=np.array([[1, 50], [1, 0]]),
    center_vertices=np.array([[2, 50], [2, 0]]),
    lanelet_id=1001
)

lanelet_shattuck_north_3 = Lanelet(
    left_vertices=np.array([[1, 50], [1, 0]]),
    right_vertices=np.array([[-1, 50], [-1, 0]]),
    center_vertices=np.array([[0, 50], [0, 0]]),
    lanelet_id=1002
)

lanelet_shattuck_north_4 = Lanelet(
    left_vertices=np.array([[-1, 50], [-1, 0]]),
    right_vertices=np.array([[-3, 50], [-3, 0]]),
    center_vertices=np.array([[-2, 50], [-2, 0]]),
    lanelet_id=1003
)

lanelet_shattuck_south_1 = Lanelet(
    left_vertices=np.array([[-3, -50], [-3, 0]]),
    right_vertices=np.array([[-5, -50], [-5, 0]]),
    center_vertices=np.array([[-4, -50], [-4, 0]]),
    lanelet_id=1004
)

lanelet_shattuck_south_2 = Lanelet(
    left_vertices=np.array([[-1, -50], [-1, 0]]),
    right_vertices=np.array([[-3, -50], [-3, 0]]),
    center_vertices=np.array([[-2, -50], [-2, 0]]),
    lanelet_id=1005
)

lanelet_shattuck_south_3 = Lanelet(
    left_vertices=np.array([[1, -50], [1, 0]]),
    right_vertices=np.array([[-1, -50], [-1, 0]]),
    center_vertices=np.array([[0, -50], [0, 0]]),
    lanelet_id=1006
)

lanelet_shattuck_south_4 = Lanelet(
    left_vertices=np.array([[3, -50], [3, 0]]),
    right_vertices=np.array([[1, -50], [1, 0]]),
    center_vertices=np.array([[2, -50], [2, 0]]),
    lanelet_id=1007
)

# I have hardcoded lanelets for Kittredge (2 eastbound, 2 westbound)
lanelet_kittredge_east_1 = Lanelet(
    left_vertices=np.array([[0, -2.5], [50, -2.5]]),
    right_vertices=np.array([[0, 0], [50, 0]]),
    center_vertices=np.array([[0, -1.25], [50, -1.25]]),
    lanelet_id=1008
)

lanelet_kittredge_east_2 = Lanelet(
    left_vertices=np.array([[0, 2.5], [50, 2.5]]),
    right_vertices=np.array([[0, 0], [50, 0]]),
    center_vertices=np.array([[0, 1.25], [50, 1.25]]),
    lanelet_id=1009
)

lanelet_kittredge_west_1 = Lanelet(
    left_vertices=np.array([[0, -2.5], [-50, -2.5]]),
    right_vertices=np.array([[0, 0], [-50, 0]]),
    center_vertices=np.array([[0, -1.25], [-50, -1.25]]),
    lanelet_id=1010
)

lanelet_kittredge_west_2 = Lanelet(
    left_vertices=np.array([[0, 2.5], [-50, 2.5]]),
    right_vertices=np.array([[0, 0], [-50, 0]]),
    center_vertices=np.array([[0, 1.25], [-50, 1.25]]),
    lanelet_id=1011
)

# Add all 12 lanelets to the scenario
scenario.lanelet_network.add_lanelet(lanelet_shattuck_south_1)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_south_2)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_south_3)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_south_4)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_north_1)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_north_2)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_north_3)
scenario.lanelet_network.add_lanelet(lanelet_shattuck_north_4)
scenario.lanelet_network.add_lanelet(lanelet_kittredge_east_1)
scenario.lanelet_network.add_lanelet(lanelet_kittredge_east_2)
scenario.lanelet_network.add_lanelet(lanelet_kittredge_west_1)
scenario.lanelet_network.add_lanelet(lanelet_kittredge_west_2)

# Function to calculate the Euclidean distance between two points
def dist(point1, point2):
    return np.sqrt((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)
# This is fine here but in general it is best we calcualte the path between 2 points and the distance of that path rather then a straght line since in tis exmapolelk opuir path is a straight line but in real life it is not.


# Define the vehicles' parameters
vehicle_shape = Rectangle(length=4.5, width=2.0)  # Typical passenger car dimensions
collisionvertex = {'x': -4, 'y': 0}
A = {'x': -4, 'y': 40}
B = {'x': -40, 'y': -1.25}
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

trajectory_states_p1 = [KSState(position=np.array([-4,max(collisionvertex['y'], 40 - i * initial_state_p1.velocity)]), velocity=10, orientation=-np.pi / 2, time_step=i) for i in range(steps)]


trajectory_states_p2 = [KSState(position=np.array([min(-40 + (i * initial_state_p2.velocity),-4), -1.25]), velocity=10, orientation=0, time_step=i) for i in range(steps)]

# Create trajectories for both vehicles
trajectory_p1 = Trajectory(initial_time_step=0, state_list=trajectory_states_p1)
trajectory_p2 = Trajectory(initial_time_step=0, state_list=trajectory_states_p2)

# Create a TrajectoryPrediction object for both vehicles
prediction_p1 = TrajectoryPrediction(trajectory=trajectory_p1, shape=vehicle_shape)
prediction_p2 = TrajectoryPrediction(trajectory=trajectory_p2, shape=vehicle_shape)

#Define the vehicles as dynamic obstacles with predictions
car1 = DynamicObstacle(
    obstacle_id=1,
    obstacle_type=ObstacleType.CAR,
    obstacle_shape=vehicle_shape,
    initial_state=initial_state_p1,
    prediction=prediction_p1
)

car2 = DynamicObstacle(
    obstacle_id=2,
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

def animate(time_step):
    ax.clear()  # Clear the plot for each frame
    rnd = MPRenderer()
    rnd.draw_params.time_begin = time_step
    scenario.draw(rnd)
    rnd.render()
    ax.set_title(f"Time Step: {time_step}")
    ax.set_xlim([-50, 50])
    ax.set_ylim([-50, 50])


ani = FuncAnimation(fig, animate, frames=len(trajectory_states_p1), interval = (1/speed)*10, repeat=False)

# Display the animation
plt.show()
