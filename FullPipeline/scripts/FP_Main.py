import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.state import KSState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle
from commonroad.common.util import Interval, AngleInterval

from FP_OSM import extractosm
from FP_OSM import convert_osm_to_commonroad
from FP_PP import create_planning_problems
from FP_PP  import center_scenario

lat = input("Enter Latitude: ").strip()
lon = input("Enter Longitude: ").strip()
radius = input("Enter Radius (in meters): ").strip()
num_party = input("Enter Number of Parties: ").strip()
num_party = int(num_party)
reigon = []
extractosm(37.868488311767578, -122.267868, 50) #("N","W")
# extractosm(37.869678497314453 , -122.265793 , 50) ("S","W")
# extractosm(37.8532791137695 , -122.2790 , 50) 

# extractosm(lat, lon, 50)
convert_osm_to_commonroad()
center_scenario()

def find_reigon(dir,mov):
    if mov == "S":
        if dir == "N":
            return "S" 
        if dir == "E":
            return "W"
        if dir == "S":
            return "N"
        if dir == "W":
            return "E"
    if mov == "TL":
        if dir == "N":
            return "W" 
        if dir == "E":
            return "N"
        if dir == "S":
            return "E"
        if dir == "W":
            return "S"
    if mov == "TR":
        if dir == "N":
            return "E" 
        if dir == "E":
            return "S"
        if dir == "S":
            return "W"
        if dir == "W":
            return "N"
        
for i in range(num_party):
    dir = input(f"Enter pre-collision dir party{i+1} (N/S/E/W)").strip()
    mov = input(f"Enter pre-collision mov party1{i+1}(S,TL,TR)").strip()
    reigon.append(find_reigon(dir,mov))



create_planning_problems(reigon)#This will take the reigon and scenario0 file and add a planningproblen to it and overwite the file.




