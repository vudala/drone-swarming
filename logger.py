import os
import logging


class Logger(logging.Logger):
    def __init__(self, log_dir: str, sub_dir: str = ""):
        """
        This class is used to output log messages to a out.log and err.log files
        out.log displays all logs, err.log displays all messages from and above
        ERROR level
    
        Parameters
        ----------
        log_dir: str
            Path to be used a root for the logging system
        sub_dir: str = ""
            Directory subtree to be created, default is empty    
        """
        super().__init__('logger_' + sub_dir, logging.DEBUG)

        # creates the file handler for out
        out_path = os.path.join(sub_dir, 'out.log')
        out_fullpath = os.path.join(log_dir, out_path)
        os.makedirs(os.path.dirname(out_fullpath), exist_ok=True)

        out = logging.FileHandler(
            filename=out_fullpath,
            mode='w',
            encoding='UTF-8'
        )
        out.setLevel(logging.DEBUG)

        # creates the file handler for err
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
