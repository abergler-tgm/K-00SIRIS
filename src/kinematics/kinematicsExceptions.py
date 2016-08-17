
class KinematicsError(Exception):
    """
    Is thrown when an error occurs within the kinematics module
    """
    def __init__(self, message):
        """
        Initializes the exception
        :param message: explanation of the error
        """

        self.message = message


class CalculationError(KinematicsError):
    """
    Is thrown when an error occurs during a calculation.
    This mostly occurs when the given point cannot be reached.
    """
    pass


class ParameterError(KinematicsError):
    """
    Is thrown if there is a problem with the given parameters.
    For example when the Base axis is the fixed axis during an inverse kinematics calculation.
    """
    pass
