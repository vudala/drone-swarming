import os
import logging


class Logger(logging.Logger):
    def __init__(self, log_dir: str, sub_dir: str = ''):
        super().__init__('logger_' + sub_dir, logging.DEBUG)

        out_path = os.path.join(sub_dir, 'out.log')
        out_fullpath = os.path.join(log_dir, out_path)
        os.makedirs(os.path.dirname(out_fullpath), exist_ok=True)

        out = logging.FileHandler(
            filename=out_fullpath,
            mode='w',
            encoding='UTF-8'
        )
        out.setLevel(logging.DEBUG)

        err_path = os.path.join(sub_dir, 'err.log')
        err_fullpath = os.path.join(log_dir, err_path)
        os.makedirs(os.path.dirname(err_fullpath), exist_ok=True)

        err = logging.FileHandler(
            filename=err_fullpath,
            mode='w',
            encoding='UTF-8'
        )
        err.setLevel(logging.ERROR)

        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
        out.setFormatter(formatter)
        err.setFormatter(formatter)

        self.addHandler(out)
        self.addHandler(err)
