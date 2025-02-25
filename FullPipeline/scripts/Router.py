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
from BhattPlannerBasic import find_traj
import xml.etree.ElementTree as ET
import random
import math
import sys
from commonroad.scenario.lanelet import Lanelet, LaneletNetwork
import json
from commonroad_route_planner.lanelet_sequence import LaneletSequence

def sample_uniform(a, b):
    """Sample uniformly between a and b."""
    return random.uniform(a, b)

def sample_rectangle(rect_elem):
    """
    Samples a point uniformly inside a rectangle defined in the commonRoad format.
    The rectangle is defined by:
      - <length>, <width>
      - <orientation> (in radians, the rotation of the rectangle)
      - <center> with <x> and <y>
    The sampling is done in the local coordinate frame of the rectangle (with x from -length/2 to length/2,
    and y from -width/2 to width/2), then rotated and translated.
    """
    length = float(rect_elem.find('length').text)
    width  = float(rect_elem.find('width').text)
    rect_orientation = float(rect_elem.find('orientation').text)
    center_elem = rect_elem.find('center')
    center_x = float(center_elem.find('x').text)
    center_y = float(center_elem.find('y').text)

    # Sample local coordinates uniformly
    local_x = sample_uniform(-length/2, length/2)
    local_y = sample_uniform(-width/2, width/2)

    # Rotate by rect_orientation and translate to global coordinates.
    global_x = center_x + local_x * math.cos(rect_orientation) - local_y * math.sin(rect_orientation)
    global_y = center_y + local_x * math.sin(rect_orientation) + local_y * math.cos(rect_orientation)
    return global_x, global_y

def sample_circle(circle_elem):
    """
    Samples a point uniformly from a circle (disk) defined in the commonRoad format.
    The circle is defined by:
      - <radius>
      - <center> with <x> and <y>
    For uniform sampling, we sample an angle uniformly [0, 2pi) and
    the radius as sqrt(u)*R where u is uniform in [0,1].
    """
    radius = float(circle_elem.find('radius').text)
    center_elem = circle_elem.find('center')
    center_x = float(center_elem.find('x').text)
    center_y = float(center_elem.find('y').text)

    angle = sample_uniform(0, 2*math.pi)
    r = math.sqrt(random.random()) * radius  # sqrt to get uniform density in the disk
    x = center_x + r * math.cos(angle)
    y = center_y + r * math.sin(angle)
    return x, y

def sample_from_interval(element):
    """
    Given an XML element that contains <intervalStart> and <intervalEnd>,
    returns a uniform random sample between the two.
    """
    start = float(element.find('intervalStart').text)
    end   = float(element.find('intervalEnd').text)
    return sample_uniform(start, end)




def find_lanelet_given_point(scenario, point):
    """
    Finds the lanelet that contains a given point.

    Args:
        scenario (Scenario): The CommonRoad scenario object.
        point (tuple): A tuple of (x, y) coordinates.

    Returns:
        Lanelet: The lanelet containing the point, or None if no lanelet contains the point.
    """
    point_geom = np.array([point[0],point[1]])
    
    for lanelet in scenario.lanelet_network.lanelets:
        if point_geom in lanelet.center_vertices:
            return lanelet
    
    return None


def planroutes(scenario_file):
    scenario, planning_problem_set= CommonRoadFileReader(scenario_file).open()
    lannetwork = scenario.lanelet_network

    for p in planning_problem_set.planning_problem_dict.values():
        initstate = p.initial_state
        init_shape = initstate.position
        init_velocity = initstate.velocity
        init_orientation = initstate.orientation

        pos = sample_rectangle(init_shape)
        velo = sample_from_interval(init_velocity)
        ori =  sample_from_interval(init_orientation) 

        start_lanelet = find_lanelet_given_point(pos)


        finstate = p.goal
        fin_shape = finstate._lanelets_of_goal_position
        fin_velocity = initstate.velocity
        fin_orientation = initstate.orientation

        pos = sample_rectangle(init_shape)
        velo = sample_from_interval(init_velocity)
        ori =  sample_from_interval(init_orientation) 

        start_lanelet = find_lanelet_given_point(pos)
        






