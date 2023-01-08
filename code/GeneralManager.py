from mission import Mission
from uuid import uuid4
import geopy.distance
import asyncio
from MAVFleetControl.mavfleetcontrol.craft import Craft 
from MAVFleetControl.mavfleetcontrol.states.position import Position
from ambulance import Ambulance


"""
This is the class that would be called from an API like application like FastAPI
"""
class GeneralManager():
    """
    Centralized Organizer of Multiple Missions
    """
    def __init__(self): 
        self.missions = []
        self.drones = []

    async def get_drone_position(self, drone):
        """
        Subsribe to each drones position
        """
        async for position in drone.inst.telemetry.position():
            drone.position = Position(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m) #relative_altitude_m
            
            # if (not self.mission_is_done):
            #     break

    async def get_ambulance_position(self, ambulance):
        """
        Subscribe to the ambulance position
        """
        await asyncio.sleep(0.2)
        ambulance.position = Position(47.3977419, 8.5455937, 488+1)
    
    def create_drone(self, id, mission_id):
        conn_address = "udp://:1454" + str(id)
        new_drone = Craft(id=id, connection_address=conn_address, mission_id= mission_id)
        return new_drone
    
    
    def create_mission(self, start_position=(55.362929,10.347584), end_position=(55.385589, 10.365061)):
        print(["[INFO] creating drones for new new mission..."])
        new_drones = []
        mission_id = uuid4()

        for n in range(self.num_drones):
            new_drone = self.create_drone(n, mission_id)
            new_drones.append(new_drone)
            self.drones.append(new_drone)

        # construct subscribe positions
        for drone in new_drones:
            """
            Ensure_future let’s us execute a coroutine in the background, without explicitly waiting for it to finish
            """
            asyncio.ensure_future(self.get_drone_position(drone))

        ambulance = Ambulance() # construct ambulance
        asyncio.ensure_future(self.get_ambulance_position(ambulance)) # subscribe position

        print(["[INFO] creating new mission..."])
        new_mission = Mission(mission_id, new_drones, ambulance, self.drones) # construct mission obj
        new_mission.setup()     # setup necessities in new mission
        self.missions.append(new_mission)
        

if __name__ == "__main__":
    manager = GeneralManager()
    manager.create_mission()


# ./Tools/simulation/gazebo/sitl_multiple_run.sh  -m iris -n 2
