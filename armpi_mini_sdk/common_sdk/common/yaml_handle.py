import yaml
lab_file_path = '/home/pi/ArmPi_mini/yaml/lab_config.yaml'
Deviation_file_path = '/home/pi/ArmPi_mini/yaml/deviation.yaml'
PickingCoordinates_file_path = '/home/pi/ArmPi_mini/yaml/picking_coordinates.yaml'

def get_yaml_data(yaml_file):
    with open(yaml_file, 'r', encoding='utf-8') as file:
        file_data = file.read()
        data = yaml.safe_load(file_data)
    return data

def save_yaml_data(data, yaml_file):
    with open(yaml_file, 'w', encoding='utf-8') as file:
        yaml.dump(data, file)
