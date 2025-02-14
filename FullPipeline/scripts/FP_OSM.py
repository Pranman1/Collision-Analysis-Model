
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet

import requests
import os
from crdesigner.map_conversion.map_conversion_interface import osm_to_commonroad
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag



# vehicle_shape = Rectangle(length=4.5, width=2.0)  # Typical passenger car dimensions
# collisionvertex = {'x': -24.3, 'y': -2.7}
# A = {'x': -27.1, 'y': 30}
# B = {'x': -47.7, 'y': -7}
# steps = 40
# speed = steps/4

# Here we will pipeline all of the sperate files into one, when executed this file will ask for:
# 1. The Location of the collison (Street 1 and Street 2) or more likeley accurate lat/long
# 2. The Number of parties 
# 3. For each party the direction of travel and their move pre coliison 
#  - Note this is a little more nuanaced there is some logic to read over here. 
#  - If pre-collision move was going straight and direcction is N we pick an S lanelet
#  - If pre-collision move was turning right and direction is N we pick an E lanelet 

def extractosm(lat,lon,radius):
      # Prompt the user for input
  
    
    # Construct the Overpass Query:
    # This query retrieves all nodes, ways, and relations around the given point and radius.
    # You can add filters for specific tags if needed.
    overpass_query = f"""
    [out:xml];
    (
      node(around:{radius},{lat},{lon});
      way(around:{radius},{lat},{lon});
      rel(around:{radius},{lat},{lon});
    );
    out body;
    >;
    out skel qt;
    """
    
    # Overpass API endpoint
    url = "https://overpass-api.de/api/interpreter"
    
    # Perform the request
    response = requests.post(url, data={'data': overpass_query})
    
    # Check if the request was successful
    if response.status_code == 200:
        # Save the resulting data to an OSM file
        filename = "Collision-Analysis-Model/FullPipeline/fullpipeline.osm"
        with open(filename, "w", encoding="utf-8") as f:
            if f == None:
                return None
            f.write(response.text)
        print(f"OSM data successfully saved to {filename}.")
    else:
        print(f"Failed to retrieve data. HTTP status code: {response.status_code}")

# Function to convert OSM data to CommonRoad XML
def convert_osm_to_commonroad():
 
    # Convert OSM to CommonRoad scenario
    file = "Collision-Analysis-Model/FullPipeline/fullpipeline.osm"
    scenario = osm_to_commonroad(file)

    # Define output file path
    output_file = "Collision-Analysis-Model/FullPipeline/scenarios/FP1.xml"

    # Save scenario to XML
    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=PlanningProblemSet(),
        author="Your Name",
        affiliation="Your Affiliation",
        source="Generated from OSM data",
        tags={Tag.URBAN},
    )
    writer.write_to_file(output_file, OverwriteExistingFile.ALWAYS)
    print(f"Scenario saved as {output_file}")