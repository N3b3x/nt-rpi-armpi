import os
import sys

# Try to import yaml with better error handling
try:
    import yaml
except ImportError as e:
    print(f"Failed to import yaml module: {e}")
    sys.exit(1)

# Get the project root directory dynamically
def get_project_root():
    # Try to find the project root by looking for the config directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up from common_sdk/common to the project root
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    return project_root

project_root = get_project_root()
yaml_dir = os.path.join(project_root, 'config')

lab_file_path_imx477 = os.path.join(yaml_dir, 'lab_config_imx477.yaml')
lab_file_path = os.path.join(yaml_dir, 'lab_config.yaml')
Deviation_file_path = os.path.join(yaml_dir, 'deviation.yaml')
PickingCoordinates_file_path = os.path.join(yaml_dir, 'picking_coordinates.yaml')

def get_yaml_data(yaml_file):
    try:
        with open(yaml_file, 'r', encoding='utf-8') as file:
            file_data = file.read()
            
        # Try different methods to load YAML data
        if hasattr(yaml, 'safe_load'):
            data = yaml.safe_load(file_data)
        elif hasattr(yaml, 'load'):
            # For older PyYAML versions, use load with SafeLoader
            if hasattr(yaml, 'SafeLoader'):
                data = yaml.load(file_data, Loader=yaml.SafeLoader)
            else:
                data = yaml.load(file_data)
        else:
            # Last resort - try with basic Loader
            data = yaml.load(file_data, Loader=yaml.Loader)
            
        return data
    except Exception as e:
        print(f"Error loading YAML file {yaml_file}: {e}")
        # Try to provide more diagnostic information
        print(f"YAML module type: {type(yaml)}")
        print(f"Available yaml methods: {[m for m in dir(yaml) if not m.startswith('_') and 'load' in m.lower()]}")
        raise

def save_yaml_data(data, yaml_file):
    try:
        with open(yaml_file, 'w', encoding='utf-8') as file:
            yaml.dump(data, file)
    except Exception as e:
        print(f"Error saving YAML file {yaml_file}: {e}")
        raise
