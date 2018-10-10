import yaml


def read_yaml(filename):
    with open(filename, 'r') as f:
        data = yaml.load(f)
    return data

def save_yaml(filename, data):
    with open(filename, 'w') as f:
        yaml.dump(data, f)
