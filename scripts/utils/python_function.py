import logging

def init_logging(log_name, log_file_path, flie_mode = 'a'):
    logging.basicConfig(format='[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s', level=logging.DEBUG, fliemode=flie_mode)
    logging.basicConfig(filename=log_file_path + log_name +".log", level=logging.DEBUG)