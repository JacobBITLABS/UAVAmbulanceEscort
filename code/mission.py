from MAVFleetControl.mavfleetcontrol.craft import Craft 
from MAVFleetControl.mavfleetcontrol.actions.goto import GoTo
from MAVFleetControl.mavfleetcontrol.actions.land import land
from droneDirection2 import DroneDirection

class Mission:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """

    def __init__(self, start_position=(55.362929,10.347584), end_position=(55.385589, 10.365061), num_drones=2): 
        self.drones = [] # 3 escorts 
        self.ambulance = None
        self.start_position = start_position
        self.end_position = end_position
        self.num_drones = num_drones

        # drones UUID/p

    def setup(self):
        print("[INFO] mission setup...")

        for n in range(self.num_drones):
            conn_address = "udp://:1454" + str(n)
            new_drone = Craft(id=n, connection_address=conn_address)
            self.drones.append(new_drone)

        # get route / waypoints 
        print("-- computing routes")
        drone_direction = DroneDirection()
        drone_direction.gmaps_init()
        
        d1, d2, route = drone_direction.get_directions(self.start_position, self.end_position)
        routes = [d1, d2]
        print("D1: ", d1)
        print("D2: ", d2)
        
        # # start the drones event loop
        # drone.start()
        for drone in self.drones:
            drone.start()
  
        # add tasks to the drones
        # 11345
        """
        Normally we would like to make a mission, however this is not suitable for fast-coordinating tasks
        """
        for drone, route in zip(self.drones, routes):
            # add way points
            for waypoint in route:
                drone.add_action(GoTo(waypoint))

                # action for wait for ambulance has passed


            # go to landing side

            # add landing 
            drone.add_action(GoTo(land()))

        # # Join the main thread
        for drone in self.drones:
            drone.join()
            
    