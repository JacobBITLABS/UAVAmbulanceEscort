from MAVFleetControl.mavfleetcontrol.states.position import Position
import geopy.distance
import time

class Ambulance:
    """
    Class Responsible for contact with Drones via the MavLink Message Protocol
    """
    def __init__(self):
        self.position = Position(55.35869, 10.34174, 488+1)
        self.waypoints = None
        self.speed = 3 #km/h, drone runs at 45 something, so be slower

    def route(self, waypoints):
        self.waypoints = [Position(float(i[1]),float(i[0]),488+1) for i in waypoints["coordinates"]]

    def dist(self, a, b):
        return geopy.distance.distance((a.lat, a.lng), (b.lat, b.lng)).km

    def drive(self):
        while self.waypoints == None:
            time.sleep(60)

        for i in self.waypoints:
            print("Ambulance goto: ", i.lat, i.lng, self.dist(self.position, i))
            to_sleep = (self.dist(self.position, i) / self.speed) * 60 * 60
            print("Ambulance sleep: ", to_sleep)
            time.sleep(to_sleep)
            self.position = i
            print("Ambulance at: ", i)
