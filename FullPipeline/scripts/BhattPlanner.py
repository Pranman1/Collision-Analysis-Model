import xml.etree.ElementTree as ET
import random
import math
import sys
import json

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

def parse_planning_problem(xml_file):
    """
    Parses the commonRoad XML file and extracts the planning problem.
    Returns a dictionary with:
      - dt: timeStepSize (global attribute)
      - init: dictionary with keys:
            time, position (x,y), orientation, velocity, yawRate, slipAngle
      - goal: dictionary with keys:
            time, position (x,y), velocity
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    dt = float(root.attrib.get("timeStepSize", "0.1"))
    
    # For simplicity we assume one planningProblem exists.
    planning_problem = root.find('planningProblem')
    if planning_problem is None:
        raise ValueError("No planningProblem element found in the XML.")

    # Parse initialState:
    init_elem = planning_problem.find('initialState')
    # Time: using <exact>
    init_time = float(init_elem.find('time/exact').text)

    # Position: check the shape type
    pos_elem = init_elem.find('position')
    # Either <rectangle> or other shapes could be handled.
    rectangle_elem = pos_elem.find('rectangle')
    if rectangle_elem is not None:
        init_x, init_y = sample_rectangle(rectangle_elem)
    else:
        raise NotImplementedError("Only rectangle shape for initial position is supported.")

    # Orientation: sample from the provided interval
    orientation_elem = init_elem.find('orientation')
    init_orientation = sample_from_interval(orientation_elem)

    # Velocity: sample from the provided interval
    velocity_elem = init_elem.find('velocity')
    init_velocity = sample_from_interval(velocity_elem)

    # YawRate and SlipAngle are given exactly.
    yaw_rate = float(init_elem.find('yawRate/exact').text)
    slip_angle = float(init_elem.find('slipAngle/exact').text)

    initial_state = {
        "time": init_time,
        "x": init_x,
        "y": init_y,
        "orientation": init_orientation,
        "velocity": init_velocity,
        "yawRate": yaw_rate,
        "slipAngle": slip_angle
    }

    # Parse goalState:
    goal_elem = planning_problem.find('goalState')
    # Time: using <intervalStart> (and assuming intervalStart == intervalEnd)
    goal_time = float(goal_elem.find('time/intervalStart').text)
    # Position: goal state position defined by a circle
    circle_elem = goal_elem.find('position/circle')
    if circle_elem is not None:
        goal_x, goal_y = sample_circle(circle_elem)
    else:
        raise NotImplementedError("Only circle shape for goal position is supported.")

    # Velocity: sample from the provided interval.
    goal_velocity_elem = goal_elem.find('velocity')
    goal_velocity = sample_from_interval(goal_velocity_elem)

    goal_state = {
        "time": goal_time,
        "x": goal_x,
        "y": goal_y,
        "velocity": goal_velocity
    }

    return {
        "dt": dt,
        "initial_state": initial_state,
        "goal_state": goal_state
    }

def interpolate_angle(theta0, theta1, s):
    """
    Linearly interpolate between two angles theta0 and theta1 taking wrapping into account.
    s is in [0, 1].
    """
    # compute the shortest angular difference:
    diff = ((theta1 - theta0 + math.pi) % (2*math.pi)) - math.pi
    return theta0 + s * diff

def plan_trajectory(init_state, goal_state, dt):
    """
    Plans a trajectory from the initial to goal state over the time horizon.
    Uses simple linear interpolation.
    
    The trajectory is a list of dictionaries, one per time step,
    with keys: time, x, y, orientation, velocity, yawRate, slipAngle.
    
    For orientation we interpolate from the sampled initial orientation to
    a “goal orientation” computed as the angle from init position to goal position.
    Velocity is linearly interpolated between initial and goal velocity.
    YawRate and slipAngle are kept constant.
    """
    T = goal_state["time"] - init_state["time"]
    steps = int(T/dt) + 1

    # Compute a "desired" goal orientation: direction from initial to goal positions.
    goal_orientation = math.atan2(goal_state["y"] - init_state["y"],
                                  goal_state["x"] - init_state["x"])

    trajectory = []
    for i in range(steps):
        t = init_state["time"] + i * dt
        s = (t - init_state["time"]) / T  # normalized time
        # Linear interpolation for x, y, and velocity.
        x = (1-s)*init_state["x"] + s*goal_state["x"]
        y = (1-s)*init_state["y"] + s*goal_state["y"]
        velocity = (1-s)*init_state["velocity"] + s*goal_state["velocity"]
        # Interpolate orientation
        orientation = interpolate_angle(init_state["orientation"], goal_orientation, s)
        # Use constant yawRate and slipAngle (exact values from init).
        state = {
            "time": round(t, 3),
            "x": round(x, 3),
            "y": round(y, 3),
            "orientation": round(orientation, 3),
            "velocity": round(velocity, 3),
            "yawRate": init_state["yawRate"],
            "slipAngle": init_state["slipAngle"]
        }
        trajectory.append(state)
    return trajectory

def main():
    if len(sys.argv) < 2:
        print("Usage: python motion_planner.py <scenario.xml>")
        sys.exit(1)
    
    xml_file = sys.argv[1]
    try:
        planning_data = parse_planning_problem(xml_file)
    except Exception as e:
        print("Error parsing XML:", e)
        sys.exit(1)
    
    dt = planning_data["dt"]
    init_state = planning_data["initial_state"]
    goal_state = planning_data["goal_state"]

    # Plan the trajectory.
    traj = plan_trajectory(init_state, goal_state, dt)
    
    # Prepare result dictionary.
    result = {
        "initial_state": init_state,
        "goal_state": goal_state,
        "trajectory": traj
    }
    
    # Output the result as JSON.
    print(json.dumps(result, indent=2))

if __name__ == '__main__':
    main()
