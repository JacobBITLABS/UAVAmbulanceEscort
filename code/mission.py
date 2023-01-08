from uuid import uuid4
from MAVFleetControl.mavfleetcontrol.craft import Craft 
from MAVFleetControl.mavfleetcontrol.actions.goto import GoTo
from MAVFleetControl.mavfleetcontrol.actions.waitForAmbulance import WaitForAmbulance
from MAVFleetControl.mavfleetcontrol.actions.land import land
from MAVFleetControl.mavfleetcontrol.states.position import Position
from droneDirection2 import DroneDirection
import asyncio

class Ambulance:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """
    def __init__(self):
        self.position = None

class Mission:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """

    def __init__(self, start_position=(55.362929,10.347584), end_position=(55.385589, 10.365061), num_drones=2): 
        self.id = uuid4()
        self.drones = [] # 3 escorts 
        self.ambulance = None # None # create ambulance object that is parsed with to the tasks with waiting for it.
        self.start_position = start_position
        self.end_position = end_position
        self.num_drones = num_drones
        self.mission_is_done = False
    

    async def get_drone_position(self, drone):
        """
        Subsribe to each drones position
        """
        async for position in drone.inst.telemetry.position():
            drone.position = Position(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m) #relative_altitude_m
            
            if (not self.mission_is_done):
                break

    async def get_ambulance_position(self, ambulance):
        """
        Subscribe to the ambulance position
        """
        await asyncio.sleep(0.2)
        ambulance.position = Position(47.3977419, 8.5455937, 488+1)

        
    def setup(self):
        print("[INFO] mission setup...")

        for n in range(self.num_drones):
            conn_address = "udp://:1454" + str(n)
            new_drone = Craft(id=n, connection_address=conn_address)
            self.drones.append(new_drone)

        # construct subscribe positions
        for drone in self.drones:
            """
            Ensure_future let’s us execute a coroutine in the background, without explicitly waiting for it to finish
            """
            asyncio.ensure_future(self.get_drone_position(drone))
        
        # Assign Ambulance 
        self.ambulance = Ambulance()

        # Set Position on Ambulance
        asyncio.ensure_future(self.get_ambulance_position(self.ambulance))
        
        # get route / waypoints 
        print("-- computing routes")
        drone_direction = DroneDirection()
        drone_direction.gmaps_init()
        
        d1, d2, route = drone_direction.get_directions(self.start_position, self.end_position)
        routes = [d1, d2]
        # print("D1: ", d1)
        # print("D2: ", d2)
        
        # # start the drones event loop
        # drone.start()
        for drone in self.drones:
            drone.start()
  
        # add tasks to the drones
        """
        Normally we would like to make a mission, however this is not suitable for fast-coordinating tasks
        """
        for drone, route in zip(self.drones, routes):
            # add way points
            for waypoint in route:
                drone.add_action(GoTo(waypoint))
                drone.add_action(WaitForAmbulance(self.ambulance))
                # action for wait for ambulance has passed

            # go to landing side depending on hospital
        
        # add landings and turn off connection
        for drone in self.drones:
            ["[INFO] mission end, drones ordered to land!"]
            drone.add_action(GoTo(land())) # land 
            drone.close_conn()             # disconnect

        # # Join the main thread
        for drone in self.drones:
            drone.join()
            
    