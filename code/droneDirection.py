from gmplot import gmplot
import googlemaps

ROUTE = [{'bounds': {'northeast': {'lat': 55.38871899999999, 'lng': 10.3650036}, 'southwest': {'lat': 55.36279709999999, 'lng': 10.3410629}}, 'copyrights': 'Map data ©2022', 'legs': [{'distance': {'text': '4.5 km', 'value': 4519}, 'duration': {'text': '8 mins', 'value': 469}, 'end_address': 'Kløvervænget 33, 5000 Odense, Denmark', 'end_location': {'lat': 55.3843306, 'lng': 10.3647379}, 'start_address': 'Lotusvej 20, 5250 Odense, Denmark', 'start_location': {'lat': 55.36279709999999, 'lng': 10.347402}, 'steps': [{'distance': {'text': '0.2 km', 'value': 192}, 'duration': {'text': '1 min', 'value': 17}, 'end_location': {'lat': 55.3639753, 'lng': 10.3496121}, 'html_instructions': 'Head <b>northeast</b> on <b>Assensvej</b>/<wbr/><b>Route 168</b>', 'polyline': {'points': 'o`|pIg~c~@MYs@wAs@{Aa@{@Wu@e@kAGQM['}, 'start_location': {'lat': 55.36279709999999, 'lng': 10.347402}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '91 m', 'value': 91}, 'duration': {'text': '1 min', 'value': 16}, 'end_location': {'lat': 55.3645632, 'lng': 10.3505776}, 'html_instructions': 'Slight <b>left</b> toward <b>Vestre Blvd.</b>', 'maneuver': 'turn-slight-left', 'polyline': {'points': '{g|pIald~@A?A?A?AAA??ACEO]a@{@_@y@Sc@'}, 'start_location': {'lat': 55.3639753, 'lng': 10.3496121}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '0.4 km', 'value': 390}, 'duration': {'text': '1 min', 'value': 51}, 'end_location': {'lat': 55.3650651, 'lng': 10.3448439}, 'html_instructions': 'Turn <b>left</b> onto <b>Vestre Blvd.</b>', 'maneuver': 'turn-left', 'polyline': {'points': 'ok|pIcrd~@CAAA?AC?A?CAC?YfAe@dB?BAB?D?JOx@AJI`AAZAFA~@BpCDbE?RFtC?^A\\@pC'}, 'start_location': {'lat': 55.3645632, 'lng': 10.3505776}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '1.3 km', 'value': 1290}, 'duration': {'text': '2 mins', 'value': 127}, 'end_location': {'lat': 55.3763471, 'lng': 10.3410629}, 'html_instructions': 'Turn <b>right</b> onto <b>Mågebakken</b>', 'maneuver': 'turn-right', 'polyline': {'points': 'un|pIgnc~@aAFM@}@L{@XIBs@\\_Ad@i@Rk@L_AJwCMo@GgBKeACG?S@G?C?I@I?[DI?i@LwDdAmCn@gCt@wDjAeDbA_DpAaDrA_@P_@V[ZMRW\\'}, 'start_location': {'lat': 55.3650651, 'lng': 10.3448439}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '1.9 km', 'value': 1913}, 'duration': {'text': '3 mins', 'value': 165}, 'end_location': {'lat': 55.38871899999999, 'lng': 10.3606199}, 'html_instructions': 'Turn <b>right</b> onto <b>Falen</b>', 'maneuver': 'turn-right', 'polyline': {'points': 'eu~pIsvb~@aA{BKKGQm@kAi@{@a@m@c@i@g@i@US[Ww@m@_NuIa@W{AaAc@_@[]a@i@[e@i@gAe@mA[gAGWOo@Ko@Ku@qAcLUkBIq@Ki@Mi@a@yASm@_@}@EUMW_@m@g@{@m@cAo@mAIMGKO[oBmDS[IMsA{Ba@q@UWMQUWMOMO_E{Gk@gAs@aBy@{Ba@sAWeA'}, 'start_location': {'lat': 55.3763471, 'lng': 10.3410629}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '0.5 km', 'value': 501}, 'duration': {'text': '1 min', 'value': 63}, 'end_location': {'lat': 55.3855928, 'lng': 10.3650036}, 'html_instructions': 'Turn <b>right</b> onto <b>J. B. Winsløws Vej</b>', 'maneuver': 'turn-right', 'polyline': {'points': 'obaqI{pf~@lCmAlB}@fB}@|BmA\\Y`@i@L]FOPu@Ha@Fm@B_AP{G'}, 'start_location': {'lat': 55.38871899999999, 'lng': 10.3606199}, 'travel_mode': 'DRIVING'}, {'distance': {'text': '0.1 km', 'value': 142}, 'duration': {'text': '1 min', 'value': 30}, 'end_location': {'lat': 55.3843306, 'lng': 10.3647379}, 'html_instructions': 'Turn <b>right</b> onto <b>Kløvervænget</b><div style="font-size:0.9em">Destination will be on the left</div>', 'maneuver': 'turn-right', 'polyline': {'points': '}n`qIglg~@`@Jz@LpARJ?NBn@@'}, 'start_location': {'lat': 55.3855928, 'lng': 10.3650036}, 'travel_mode': 'DRIVING'}], 'traffic_speed_entry': [], 'via_waypoint': []}], 'overview_polyline': {'points': 'o`|pIg~c~@wCiGsAoDGAUe@yA{CGCGA_AlDAXQdAK|ACfAHtIFhE?nDoAH}@L{@X}@`@iBx@k@L_AJwCMo@GgBKeAC[@K?y@Fi@LwDdAuGdB}InCaIdD_Ah@i@n@W\\aA{BKKGQwAgCeAwA}@}@sAeAaOmJ{AaAc@_@}@gAeAmBaAuCWgAWeBgBoOU{Ao@cCs@kBEUMWgAiB}AqCqCcFsCwEgAqAmEkH_BiDy@{Ba@sAWeAlCmAtE{B|BmA\\Y`@i@L]XeAPoAT{I|AX|AR~@D'}, 'summary': 'Mågebakken and Falen', 'warnings': [], 'waypoint_order': []}]

class DroneDirection:
    """
    Class Responsible for communication with Google Maps Geolocation and Directions API
    """
    def __init__(self, api_key='AIzaSyCkg3-WSTJJQbwswxJnQ_QN2_VtbtZS_t0', num_drones=2):
        # init class attributes
        self.api_key = api_key
        self.num_drones = num_drones
        self.gmaps = None
    
    def gmaps_init(self):
        """ Construct google maps client object """
        #api_key = 'AIzaSyCkg3-WSTJJQbwswxJnQ_QN2_VtbtZS_t0'
        self.gmaps = googlemaps.Client(key=self.api_key)


    def plot_waypoints(self, start_location, end_location, step_coordinates):
        """ plot route waypoints """
        print("STEP COORDS")
        print(step_coordinates)
        # Create the map plotter:
        apikey = self.api_key # (your API key here)
        lat_start = start_location['lat']
        lng_start = start_location['lng']
        lat_end = end_location['lat']
        lng_end = end_location['lng']

        gmap = gmplot.GoogleMapPlotter(lat_start, lng_start, 14, apikey=apikey)

        # Highlight Actions / Drone waypoints:
        attractions_lats, attractions_lngs = zip(*step_coordinates)
        color = ['red'] * len(step_coordinates)
        gmap.scatter(attractions_lats, attractions_lngs, color=color, size=40, marker=True)

        gmap.directions(
            (lat_start, lng_start),
            (lat_end, lng_end),
            waypoints = step_coordinates
        )

        # Draw the map to an HTML file:
        gmap.draw('map.html')


    def get_direction(self, origin=(55.362856, 10.347292), dest=(55.384325, 10.364990), plot_map = False):
        """ Get direction from origin and destination sets of lat/lon"""
        print("Get direction")
        route = ROUTE # self.gmaps.directions(origin=origin, destination=dest, mode='driving', alternatives=False)
        legs = route[0]['legs'][0]
        start_location = legs['start_location'] # Start location
        end_location = legs['end_location'] # end location
        steps = legs['steps'] 

        steps_coordinates = [] 
        for step in steps:
            # # Handle end_location
            # location = step['start_location']
            # lat = location['lat']
            # lng = location['lng']
            # steps_coordinates.append((lat, lng))

            # Handle end_location
            location = step['end_location']
            lat = location['lat']
            lng = location['lng']
            steps_coordinates.append((lat, lng))

        # plot route 
        if plot_map:
            self.plot_waypoints(start_location, end_location, steps_coordinates)

        # divide the steps steps_coordinates into chuncks the number of drones
        def split_list(a_list):
            """ Splits lists into 2 chuncks"""
            half = len(a_list)//2
            return a_list[:half], a_list[half:]

        B, C = split_list(steps_coordinates)
        return B, C


def main():
    print("MAIN")
    drone_direction = DroneDirection()
    drone_direction.gmaps_init()
    drone_0, drone_1 = drone_direction.get_direction()
    print(drone_0)
    print(drone_1)



main()
    

    