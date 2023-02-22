""" Module to Aerostack UI logs """

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class AerostackUILogger:
    """ Class to manage Aerostack UI logs """

    def __init__(self, log_level: int = 0):
        """ Initialize the logger
        Args:
            log_level (int): 0 = no logs,
                             1 = error logs,
                             2 = error and warnings logs
                             3 = error, warnings and info logs
                             4 = error, warnings, info and debug logs
        """
        print("Initializing Aerostack UI logger: " + str(log_level))
        self.log_level = log_level
        self.error_enable = self.log_level >= 1
        self.warning_enable = self.log_level >= 2
        self.info_enable = self.log_level >= 3
        self.debug_enable = self.log_level >= 4

    def get_log_level(self) -> int:
        """ Get the log level """
        return self.log_level

    def error(self, class_name: str, function: str, message: str) -> None:
        """ Log an error message """
        if self.error_enable:
            print(f"{bcolors.FAIL}ERROR: {class_name} - {function} - {message}{bcolors.ENDC}")

    def warning(self, class_name: str, function: str, message: str) -> None:
        """ Log a warning message """
        if self.warning_enable:
            print(f"{bcolors.WARNING}WARNING: {class_name} - {function} - {message}{bcolors.ENDC}")

    def info(self, class_name: str, function: str, message: str) -> None:
        """ Log an info message """
        if self.info_enable:
            print(f"{bcolors.OKGREEN}INFO: {class_name} - {function} - {message}{bcolors.ENDC}")

    def debug(self, class_name: str, function: str, message: str) -> None:
        """ Log a debug message """
        if self.debug_enable:
            print(f"{bcolors.OKBLUE}DEBUG: {class_name} - {function} - {message}{bcolors.ENDC}")
