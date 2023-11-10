import logging
import yaml

def init_logging(log_name, log_file_path, filemode = 'a', level = logging.INFO):
    logging.basicConfig(filename=log_file_path + log_name +".log",
                        format='[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s',
                        level=level,
                        filemode=filemode)

def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data