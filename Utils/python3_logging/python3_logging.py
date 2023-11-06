import logging

def init_logging(log_name:str, log_file_path):
    log_format = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    logging.basicConfig(
        level=logging.DEBUG,
        format=log_format,
        filename=log_name+'.log',
        filemode='w'
    )
    file_handler = logging.FileHandler(log_file_path)
    formatter = logging.Formatter(log_format)
    file_handler.setFormatter(formatter)
    logging.getLogger(log_name).addHandler(file_handler)