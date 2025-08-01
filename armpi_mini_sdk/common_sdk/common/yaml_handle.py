import os
import sys
import re

# Try to import PyYAML with better error handling
yaml = None
try:
    import yaml
    # Verify it's actually PyYAML by checking for characteristic methods
    if not (hasattr(yaml, 'safe_load') or hasattr(yaml, 'load') or hasattr(yaml, 'dump')):
        print("Warning: Imported yaml module doesn't have expected methods. This might be a local yaml.py file.")
        yaml = None
except ImportError:
    print("PyYAML not found. Will use basic YAML parser.")
    yaml = None

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

def basic_yaml_parse(content):
    """Basic YAML parser for simple key-value pairs"""
    result = {}
    for line in content.split('\n'):
        line = line.strip()
        if line and not line.startswith('#'):
            # Match key: value pattern
            match = re.match(r'^([^:]+):\s*(.+)$', line)
            if match:
                key = match.group(1).strip().strip("'\"")
                value = match.group(2).strip()
                # Try to convert value to appropriate type
                try:
                    if value.lower() in ('true', 'false'):
                        result[key] = value.lower() == 'true'
                    elif '.' in value:
                        result[key] = float(value)
                    else:
                        result[key] = int(value)
                except ValueError:
                    result[key] = value.strip("'\"")
    return result

def get_yaml_data(yaml_file):
    try:
        with open(yaml_file, 'r', encoding='utf-8') as file:
            file_data = file.read()
            
        if yaml is not None:
            # Use PyYAML if available
            if hasattr(yaml, 'safe_load'):
                data = yaml.safe_load(file_data)
            elif hasattr(yaml, 'load'):
                if hasattr(yaml, 'SafeLoader'):
                    data = yaml.load(file_data, Loader=yaml.SafeLoader)
                else:
                    data = yaml.load(file_data)
            else:
                data = yaml.load(file_data, Loader=yaml.Loader)
        else:
            # Use basic parser as fallback
            data = basic_yaml_parse(file_data)
            
        return data
    except Exception as e:
        print(f"Error loading YAML file {yaml_file}: {e}")
        if yaml is not None:
            print(f"YAML module type: {type(yaml)}")
            print(f"YAML module file: {getattr(yaml, '__file__', 'Unknown')}")
            print(f"Available yaml methods: {[m for m in dir(yaml) if not m.startswith('_') and 'load' in m.lower()]}")
        else:
            print("Using basic YAML parser fallback")
        raise

def save_yaml_data(data, yaml_file):
    try:
        if yaml is not None:
            with open(yaml_file, 'w', encoding='utf-8') as file:
                yaml.dump(data, file)
        else:
            # Basic YAML writer
            with open(yaml_file, 'w', encoding='utf-8') as file:
                for key, value in data.items():
                    file.write(f"{key}: {value}\n")
    except Exception as e:
        print(f"Error saving YAML file {yaml_file}: {e}")
        raise
