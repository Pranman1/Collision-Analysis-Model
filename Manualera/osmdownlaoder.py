import os
from crdesigner.map_conversion.map_conversion_interface import osm_to_commonroad
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Tag

# Base directory as per your structure
base_dir = "/root/Commonroad"
output_folder = os.path.join(base_dir, "output")
osm_folder = os.path.join(base_dir, "osm_files")
os.makedirs(output_folder, exist_ok=True)
os.makedirs(osm_folder, exist_ok=True)

# Input parameters
street_name_1 = "Kittredge Street"
street_name_2 = "Shattuck Avenue"
radius = 500  # Radius in meters

# Function to convert OSM data to CommonRoad XML
def convert_osm_to_commonroad(street1, street2, search_radius, osm_dir, output_dir):
    # Define the center point (latitude and longitude) for the OSM data
    # Placeholder values for latitude and longitude; replace with geocoding if needed
    center_lat = 37.868488311767578 
  # Latitude for Berkeley, CA
    center_lon = -122.267868 
  # Longitude for Berkeley, CA

    # Download OSM data around the center point
    # osm_file = "fullpipeline.osm"
    # download_command = (
    #     f"osmconvert --out-osm -b={center_lon - 0.005},{center_lat - 0.005},"
    #     f"{center_lon + 0.005},{center_lat + 0.005} -o={osm_file}"
    # )
    # os.system(download_command)

    # Convert OSM to CommonRoad scenario
    file = "Commonroad/osm_files/map.osm"
    scenario = osm_to_commonroad(file)

    # Define output file path
    output_file = os.path.join(output_dir, "Trial.xml")

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

# Run the conversion
convert_osm_to_commonroad(street_name_1, street_name_2, radius, osm_folder, output_folder)
