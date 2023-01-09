from MAVFleetControl.mavfleetcontrol.states.position import Position

class Ambulance:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """
    def __init__(self):
        self.position = Position(47.3977419, 8.5455937, 488+1)
