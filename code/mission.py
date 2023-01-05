
class Mission:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """
    
    def __init__(self): 
        drones = [] # 3 escorts 
        ambulance = None

        # drones UUID/p