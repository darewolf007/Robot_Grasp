import logging
import yaml

def init_logging(log_name, log_file_path, filemode='a', level=logging.INFO):
    # 创建一个名为 log_name 的日志记录器
    logger = logging.getLogger(log_name)
    # 设置日志级别
    logger.setLevel(level)
    # 创建格式化器
    formatter = logging.Formatter('[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s')
    # 创建文件处理器，将日志写入文件
    file_handler = logging.FileHandler(log_file_path + log_name + ".log", mode=filemode)
    file_handler.setFormatter(formatter)
    # 将处理器添加到日志记录器
    logger.addHandler(file_handler)


def read_yaml_file(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data