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

import matplotlib.pyplot as plt

# def download_osm_map(street1, street2, lat, lng, radius, filename):
#     """
#     Downloads an OSM street map centered around the intersection of two streets
#     near the provided latitude and longitude. If the intersection isn't found,
#     it uses the given coordinates.

#     Parameters:
#     - street1 (str): Name of the first street.
#     - street2 (str): Name of the second street.
#     - lat (float): Latitude of the approximate location.
#     - lng (float): Longitude of the approximate location.
#     - radius (float): Radius in meters for the area to download.
#     - filename (str): Filename to save the map image.

#     Returns:
#     - None
#     """
#     # Attempt to find the intersection of the two streets near the coordinates
#     try:
#         query = f"{street1} & {street2}, near ({lat}, {lng})"
#         point = ox.geocode(query)
#     except Exception as e:
#         print(f"Geocoding failed: {e}")
#         print("Using provided latitude and longitude instead.")
#         point = (lat, lng)
    
#     # Download the street network within the specified radius
#     G = ox.graph_from_point(point, dist=radius, network_type='drive')

#     # Plot the street network
#     fig, ax = ox.plot_graph(G, show=False, close=False)

#     # Save the plot to a file
#     plt.savefig(filename, dpi=300, bbox_inches='tight')
#     plt.close()
#     print(f"Map saved to {filename}")




# Function to convert OSM data to CommonRoad XML
def convert_osm_to_commonroad(street1, street2, search_radius, osm_dir, output_dir):
    # Define the center point (latitude and longitude) for the OSM data
    # Placeholder values for latitude and longitude; replace with geocoding if needed
    center_lat = 37.868488311767578   # Latitude for Berkeley, CA
    center_lon = -122.267868   # Longitude for Berkeley, CA

    # Download OSM data around the center point
    osm_file = os.path.join(osm_dir, "demomap.osm")
    download_command = (
        f"osmconvert --out-osm -b={center_lon - 0.005},{center_lat - 0.005},"
        f"{center_lon + 0.005},{center_lat + 0.005} -o={osm_file}"
    )
    os.system(download_command)

    # Convert OSM to CommonRoad scenario
    scenario = osm_to_commonroad(osm_file)

    # Define output file path
    output_file = os.path.join(output_dir, "map2.xml")

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
# download_osm_map('5th Avenue', 'West 34th Street', 40.748817, -73.985428, 500, 'map.png')
convert_osm_to_commonroad(street_name_1, street_name_2, radius, osm_folder, output_folder)
