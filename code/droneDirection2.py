import openrouteservice
from openrouteservice import convert
from OSMPythonTools.overpass import Overpass
from OSMPythonTools.api import Api
from gmplot import gmplot
import googlemaps


class DroneDirection: 
    """
    Class Responsible for communication with OSM and Google Maps For Plotting
    """
    def __init__(self, api_key='PROVIDE GOOGEL MAPS KEY HERE', num_drones=2):
        # init class attributes
        self.api_key = api_key
        self.num_drones = num_drones
        self.gmaps = None

    
    def get_directions(self, start=(55.362929,10.347584), end=(55.385589, 10.365061)):
        decoded, coords = self.get_route(start=start, plot_map=False)
        step_coordinates = self.get_intersections(decoded, coords, plot_map=True)
        B, C = DroneDirection.split_list(step_coordinates)

        return B, C, decoded

    def gmaps_init(self):
        """ Construct google maps client object """
        #api_key = 'AIzaSyCkg3-WSTJJQbwswxJnQ_QN2_VtbtZS_t0'
        self.gmaps = googlemaps.Client(key=self.api_key)


    def plot_waypoints(self, start_location, end_location, step_coordinates, color='red'):
        """ plot route waypoints """
        # Create the map plotter:
        apikey = self.api_key # (your API key here)
        lat_start = start_location[0]
        lng_start = start_location[1]
        lat_end = end_location[0]
        lng_end = end_location[1]

        gmap = gmplot.GoogleMapPlotter(lat_start, lng_start, 14, apikey=apikey)

        # Highlight Actions / Drone waypoints:
        attractions_lats, attractions_lngs = zip(*step_coordinates)
        color = [color] * len(step_coordinates)
        gmap.scatter(attractions_lats, attractions_lngs, color=color, size=40, marker=True)

        gmap.directions(
            (lat_start, lng_start),
            (lat_end, lng_end),
            waypoints = step_coordinates
        )
        # Draw the map to an HTML file:
        gmap.draw('map.html')

    def get_route(self, start=(55.362929,10.347584), end=(55.385589, 10.365061) , plot_map = False):
        """
        Compute the Route
        """
        # API expect lat/lng flipped in tuples
        coords = ((start[1], start[0]), (end[1], end[0]))
        client = openrouteservice.Client(key='PROVIDE OPEN ROUTE SERVICE KEY HERE') # Specify your personal API key
        # decode_polyline needs the geometry only
        geometry = client.directions(coords)['routes'][0]['geometry']
        decoded = convert.decode_polyline(geometry) # line string 
        coordinates = decoded['coordinates'] # PLOT As
        
        steps_coordinates = []
        for cord in coordinates:
            lat = cord[1]   
            lng = cord[0] 
            steps_coordinates.append((lat, lng))
        
        if plot_map:
            #print("Plotting Map")
            start_location = (coords[0][1], coords[1][0])
            end_location = (coords[1][1], coords[1][1])
            self.plot_waypoints(start_location, end_location, steps_coordinates)
            
        return decoded, coords


    def get_intersections(self, decoded, coords, plot_map=False):
        """
        decoded: polyline geometry
        
        > Get the intersections with traffic-signals within the distance 8 of polyline
        > Sort the noise (the onces that is misplaces on map)
        > Sort so there is only one node per intersection. This is a result of that a way is multilanes
        """
        overpass = Overpass()
        coordinates = decoded["coordinates"]
        linestring = ''
        first = True # hack make smarter
        for coord in coordinates:
            lng = coord[0]
            lat = coord[1]
            # hack just remove first character in str
            if first:
                next_string = str(lat) + "," + str(lng)
                first = False
            else: 
                next_string = ',' + str(lat) + "," + str(lng)
            linestring += next_string

        #query = 'way[highway="tertiary"](around:10,' + linestring + ');' + ' (._;>;);' +  'out meta;'
        query = 'node[highway="traffic_signals"](around:8,' + linestring + ');' + ' (._;>;);' +  'out meta;'
   
        # Create Query
        results = overpass.query(query)
        steps_coordinates = []
        for node in  results.nodes():
            node_tags = node.tags()
            # print("node:tags: ", node_tags)
            # print(node.geometry())
            if node_tags:
                # test keys that can be a signal
                if 'traffic_signals' in node_tags or 'crossing' in node_tags:
                    geom_node = node.geometry()
                    node_coord = geom_node['coordinates']
                    lat = node_coord[1]   
                    lng = node_coord[0] 
                    steps_coordinates.append((lat, lng))

        # Filter Coordinates
        sorted_step_coordinates = sorted(steps_coordinates)
        # print(sorted_step_coordinates)
        i = 0
        while(i != len(sorted_step_coordinates)-2):
            lat, lng = sorted_step_coordinates[i] # current 
            lat_1, lng_1 = sorted_step_coordinates[i+1]

            if abs(lat-lat_1) < 0.001 and abs(lng-lng_1) < 0.001:     # if diff is less than 
                del sorted_step_coordinates[i+1] # remove coordinate from list
            else:
                # move on to next coordinate
                i +=1        

        if plot_map:
            # print("Plotting Map")
            coords = ((10.347584, 55.362929), (10.365061, 55.385589))
            start_location = (coords[0][1], coords[1][0])
            end_location = (coords[1][1], coords[1][1])
            self.plot_waypoints(start_location, end_location, sorted_step_coordinates, color='blue')

        
        print(sorted_step_coordinates)
        return sorted_step_coordinates

     
    def split_list(a_list):
        """ Splits lists into 2 chuncks"""
        half = len(a_list)//2
        return a_list[:half], a_list[half:]



def main():
    print("Drone Direction")
    drone_direction = DroneDirection()
    drone_direction.gmaps_init()
    
    d1, d2, route = drone_direction.get_directions(start=(55.367651, 10.411133), end=(55.385589, 10.365061))
    print("D1: ", d1)
    print("D2: ", d2)
  


main()
# UAVAmbulanceEscort