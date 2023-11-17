import logging
import yaml

def init_logging(log_name, log_file_path, filemode='a', level=logging.INFO):
    logger = logging.getLogger(log_name)
    logger.setLevel(level)
    formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s')
    file_handler = logging.FileHandler(log_file_path + log_name + ".log", mode=filemode)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)


def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data