from mission import Mission
import geopy.distance
import asyncio

"""
This is the class that would be called from an API like application like FastAPI
"""
class GeneralManager():
    """
    Centralized Organizer of Multiple Missions
    """
    def __init__(self): 
        self.missions = []
    
    def create_mission(self, start_position=(55.362929,10.347584), end_position=(55.385589, 10.365061)):
        print(["[INFO] creating new mission..."])
        new_mission = Mission() # construct mission obj
        new_mission.setup()     # setup necessities in new mission
        self.missions.append(new_mission)


    def get_drone_positions():
        pass

if __name__ == "__main__":
    manager = GeneralManager()
    manager.create_mission()


# ./Tools/simulation/gazebo/sitl_multiple_run.sh  -m iris -n 2
