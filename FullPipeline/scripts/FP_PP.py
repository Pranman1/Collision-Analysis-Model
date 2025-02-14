from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.common.util import Interval, AngleInterval
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.lanelet import Lanelet
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep
from scipy.spatial import KDTree
from commonroad.common.file_writer import CommonRoadFileWriter,OverwriteExistingFile
import math
from shapely.geometry import Point
from itertools import product

def find_orientation(lanelet,pt):
    return

def ret_rectangle(center,ori):
    return Rectangle(width = 2, length = 4, center=np.array(center), orientation= ori)


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


goal_state = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=30.0, center=np.array([0,0])),
    velocity=Interval(1.0, 55.0)
)

goal_region = GoalRegion([goal_state])

def dist(x,y):
    return math.sqrt(x**2 + y**2)

def reigon(x,y):
    if max(-x-y,x-y)<=0:
        return "N"
    if max(-x-y,-x+y)<=0:
        return "E"
    if max(x+y,-x+y)<=0:
        return "S"
    if max(x+y,x-y)<=0:
        return "W"
    
def dirori(lanelet, pt):
    pi = math.pi
    a = lanelet.orientation_by_position(pt)
    if a<=3*pi/4 and a>= pi/4:
        return "S"
    if a<=pi/4 and a>= -pi/4:
        return "W"
    if a<= -pi/4 and a>= -3*pi/4:
        return "N"
    if (a<= pi and a>= 3*pi/4) or (a>=-pi and a<=-3*pi/4):
        return "E"


def center_scenario():
    scenario_file =  "Collision-Analysis-Model/FullPipeline/scenarios/FP1.xml"
    scenario,_= CommonRoadFileReader(scenario_file).open()
    center_points = []

    for lanelet in scenario.lanelet_network.lanelets:
        center_points.extend(lanelet.center_vertices)
       
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

    scenario.translate_rotate(-densest_point,0)
    scenario_file =  "Collision-Analysis-Model/FullPipeline/scenarios/FPShift.xml"
    planning_problem_set = _

    CommonRoadFileWriter(scenario,planning_problem_set).write_scenario_to_file(scenario_file, OverwriteExistingFile.ALWAYS)





def generate_planning_problem_permutations(planning_problem_lists):
    """
    Generate all permutations of planning problems where each permutation
    contains one planning problem from each of the initial sublists.

    Args:
        planning_problem_lists (list of list): A list of sublists where each sublist contains planning problems.

    Returns:
        list of list: A list of lists, where each inner list represents a permutation of planning problems.
    """
    # Use itertools.product to generate all combinations
    permutations = list(product(*planning_problem_lists))
    
    # Convert each tuple to a list
    return [list(permutation) for permutation in permutations]

def create_planning_problems(reigions):
    scenario_file =  "Collision-Analysis-Model/FullPipeline/scenarios/FPShift.xml"
    scenario,pp= CommonRoadFileReader(scenario_file).open()
    if pp == None:
        pp = PlanningProblemSet()

    center_points = []
    for lanelet in scenario.lanelet_network.lanelets:
        a = len(lanelet.center_vertices)//2
        center_points.append(lanelet.center_vertices[a])


    J = 0
    all_pp =[]
    for i in range(len(reigions)):
        temp =[]
        for a in center_points:
            x = a[0]
            y = a[1]

            if dist(x,y)>20:

                lan = find_lanelet_given_point(scenario,(x,y))
                
                ra = lan.center_vertices
                pt1 = ra[0]
                b = len(ra)//2
                pt2 = ra[b]
            
                if reigon(x,y) == reigions[i] and dirori(lan,pt2) == reigions[i] :

                    ra = lan.center_vertices
                    pt1 = ra[0]
                    b = len(ra)//2
                    pt2 = ra[b]
                    # ori2 = lan.orientation_by_position(pt2)

                    ori2 = math.atan((pt1[0]-pt2[0])/(pt1[1]-pt2[1]))
                    startshape = ret_rectangle([x,y],ori2)
                    print(ori2, f"orinetatiomn for {J}, and {i}")

                    start_state = InitialState(
                        time_step=0,
                        position= startshape,
                        velocity=Interval(10.0,30),
                        orientation = Interval(ori2-0.01, ori2 + 0.01),
 
                        # orientation = AngleInterval(-(3.141),(3.141)),  # Heading angle in radians
                        yaw_rate=0.0,  # Rotational velocity
                        slip_angle=0.0  # Slip angl
                        )
                    planning_problem = PlanningProblem(J, initial_state=start_state, goal_region=goal_region)

                    # pp.add_planning_problem(planning_problem)
                    temp.append(planning_problem)
                    J+=1
        all_pp.append(temp)
    
    print(all_pp) 
    t = 0


    perm = generate_planning_problem_permutations(all_pp)

    i = 0
    for lst in perm:
        scenario_file =  f"Collision-Analysis-Model/FullPipeline/output/OUT{i}.xml"
        pp = PlanningProblemSet()
        for j in lst: 
            pp.add_planning_problem(j)
        i+=1
        
                     
        CommonRoadFileWriter(scenario,pp).write_to_file(scenario_file, OverwriteExistingFile.ALWAYS)


create_planning_problems(["N"])
