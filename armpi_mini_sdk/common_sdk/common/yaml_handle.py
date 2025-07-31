import yaml
import os

# Get the project root directory dynamically
def get_project_root():
    # Try to find the project root by looking for the yaml directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up from common_sdk/common to the project root
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    return project_root

project_root = get_project_root()
yaml_dir = os.path.join(project_root, 'yaml')

lab_file_path_imx477 = os.path.join(yaml_dir, 'lab_config_imx477.yaml')
lab_file_path = os.path.join(yaml_dir, 'lab_config.yaml')
Deviation_file_path = os.path.join(yaml_dir, 'deviation.yaml')
PickingCoordinates_file_path = os.path.join(yaml_dir, 'picking_coordinates.yaml')

def get_yaml_data(yaml_file):
    with open(yaml_file, 'r', encoding='utf-8') as file:
        file_data = file.read()
        data = yaml.safe_load(file_data)
    return data

def save_yaml_data(data, yaml_file):
    with open(yaml_file, 'w', encoding='utf-8') as file:
        yaml.dump(data, file)
