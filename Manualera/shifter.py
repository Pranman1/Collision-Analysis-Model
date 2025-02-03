import xml.etree.ElementTree as ET

# Load the XML file
file_path = 'Commonroad/output/map2.xml'
tree = ET.parse(file_path)
root = tree.getroot()

# Shift values
x_shift = 13610768.9405
y_shift = -4560874.9279

# Traverse and modify <x> and <y> elements
for elem in root.iter():
    if elem.tag == 'x':
        original_value = float(elem.text)
        elem.text = str(original_value + x_shift)
    elif elem.tag == 'y':
        original_value = float(elem.text)
        elem.text = str(original_value + y_shift)

# Save the modified XML to a new file
tree.write('shifted_file6.xml')