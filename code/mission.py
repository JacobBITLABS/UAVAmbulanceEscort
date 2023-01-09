from MAVFleetControl.mavfleetcontrol.actions.goto import GoTo
from MAVFleetControl.mavfleetcontrol.actions.waitForAmbulance import WaitFor
from MAVFleetControl.mavfleetcontrol.actions.land import land
from MAVFleetControl.mavfleetcontrol.craft import State
from droneDirection2 import DroneDirection

class Mission:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """
    def __init__(self, id, drones, ambulance, all_drones, start_position=(55.362929,10.347584), end_position=(55.385589, 10.365061), num_drones=2): 
        self.id = id
        self.drones = drones # 3 escorts 
        self.all_drones = all_drones # reference to all drones
        self.ambulance = ambulance # None # create ambulance object that is parsed with to the tasks with waiting for it.
        self.start_position = start_position
        self.end_position = end_position
        self.num_drones = num_drones
        self.mission_is_done = False
    

    def setup(self):
        """
        Setup a mission
        """
        print("[INFO] mission setup...")     
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
            drone.state = State.Start # set drone state
  
        # add tasks to the drones
        """
        Normally we would like to make a mission, however this is not suitable for fast-coordinating tasks
        """
        for drone, route in zip(self.drones, routes):
            # add way points
            for waypoint in route:
                drone.add_action(GoTo(waypoint))
                drone.add_action(WaitFor(self.ambulance, self.all_drones))
            
            # go to landing side depending on hospital
        
        # add landings and turn off connection
        for drone in self.drones:
            ["[INFO] mission end, drones ordered to land!"]
            drone.add_action(GoTo(land())) # land 
            drone.state = State.End        # End drone state, stop background position tracking
            drone.close_conn()             # disconnect


        # # Join the main thread
        for drone in self.drones:
            drone.join()
            
    