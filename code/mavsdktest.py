#$ cd ~/PX4-Autopilot/Tools/simulation/gazebo
#$ ./sitl_multiple_run.sh -n 3
import asyncio
import geopy.distance
from enum import Enum
from mavsdk import System
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)

#State of the drone
class State(Enum):
	Start = 1 #Startup sequence
	Follow = 2
	FollowMission = 3
	Travel = 4 #Normal travel
	TravelWait = 5 #Is decending to waypoint
	Wait = 6 #Is at a waypoint
	End = 7 #At the end

class Drone:
	inst = None
	pos = None
	speed = None
	idx = 0
	rank = 0
	mission = 0
	state = State.Start
	def __init__(self, inst, pos, idx, rank) -> None:
		self.inst = inst
		self.pos = pos
		self.idx = idx
		self.rank = rank

class Mission:
	idx = 0
	pos = None
	stop = False
	visited = False
	def __init__(self, idx, pos, stop) -> None:
		self.idx = idx
		self.pos = pos
		self.stop = stop

class Pos:
	latitude_deg = 0
	longitude_deg = 0
	altitude_m = 0
	def __init__(self, lat, long, alt) -> None:
		self.latitude_deg = lat
		self.longitude_deg = long
		self.altitude_m = alt

drones = []
offground_height = 8 #How far up should we be?
mission = [
	Mission(0, Pos(47.3977419, 8.5455937, 488+offground_height), False),
	Mission(1, Pos(47.397459473771, 8.546179442458623, 488+offground_height), True),
	Mission(2, Pos(47.39707095615432, 8.54589240348472, 488+offground_height), True),
	Mission(3, Pos(47.3970510263338, 8.546029192499857, 488+offground_height), True),
	Mission(4, Pos(47.39741230621734, 8.546302785162757, 488+offground_height), True)
]
running = True
offset_height = 10


default_height = 8.0 #in Meters
follow_distance = 2.0 #in Meters, this is the distance that the drone will remain away from Target while following it
#Direction relative to the Target
#Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
direction = Config.FollowDirection.BEHIND
responsiveness =  0.02


def get_ambulance_pos():
	return [47.3977419, 8.5455937]

def dist(a,b):
	return geopy.distance.geodesic((a.latitude_deg, a.longitude_deg), (b.latitude_deg, b.longitude_deg)).m

async def print_position(idx):
	global drones
	global running

	async for position in drones[idx].inst.telemetry.position():
		drones[idx].pos = Pos(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m) #relative_altitude_m
		if (not running):
			break

async def print_velocity(idx):
	global drones
	global running

	async for odom in drones[idx].inst.telemetry.position_velocity_ned():
		drones[idx].speed = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]
		if (not running):
			break

#FollowMe is buggy in the simulator, for some reason
follow_me_enable = False

async def run():
	global drones
	global running

	for i in range(3):
		drones.append(
			Drone(
				System(port=50040 + i),
				[],
				i,
				i
			)
		)

	print("-- Connecting")
	for k,drone in enumerate(drones):
		await drone.inst.connect(system_address=f"udp://:{14540 + k}")

	print("-- Waiting for drone to connect...")
	for drone in drones:
		async for state in drone.inst.core.connection_state():
			if state.is_connected:
				print(f"-- Connected to drone!")
				break

	for k,drone in enumerate(drones):
		asyncio.ensure_future(print_position(k))
		asyncio.ensure_future(print_velocity(k))

	print("-- Arming")
	for drone in drones:
		await drone.inst.action.arm()

	if (follow_me_enable):
		print("-- Set follow_me config")
		for drone in drones:
			#TODO: Fails for some reason?
			conf = Config(default_height, follow_distance, direction, responsiveness)
			await drone.inst.follow_me.set_config(conf)

	print("-- Taking off")
	for drone in drones:
		await drone.inst.action.takeoff()

	await asyncio.sleep(40)

	print("-- Loop")
	while (running):
		for k,drone in enumerate(drones):
			#Start state
			if drone.state == State.Start:
				#If first drone
				if (drone.idx == 0):
					print("Drone start!")
					#Follow mission
					m = mission[drone.mission]
					await drone.inst.action.goto_location(
						m.pos.latitude_deg,
						m.pos.longitude_deg,
						m.pos.altitude_m + offset_height,
						0,
					)
					drone.state = State.Travel
				else:
					#Follow other drone
					if (follow_me_enable):
						await drone.inst.follow_me.start()
					drone.state = State.Follow
				
			#Follow next drone in order (keep a line)
			if drone.state == State.Follow:
				next_drone = [i for i in drones if i.rank == drone.rank-1][0]
				#If same mission (dont go diagonal through buildings, ever)
				if (next_drone.mission == drone.mission):
					if (follow_me_enable):
						target = TargetLocation(next_drone.pos.latitude_deg, next_drone.pos.longitude_deg, next_drone.pos.altitude_m, 0, 0, 0)
						await drone.inst.follow_me.set_target_location(target)
					else:
						await drone.inst.action.goto_location(
							next_drone.pos.latitude_deg, #TODO: Behind somehow
							next_drone.pos.longitude_deg,
							next_drone.pos.altitude_m,
							0,
						)
				else: #If not same mission, go to mission checkpoint instead
					await drone.inst.follow_me.stop()
					drone.state = State.FollowMission

			#Go to mission checkpoint
			if drone.state == State.FollowMission:
				m = mission[drone.mission]
				await drone.inst.action.goto_location(
					m.pos.latitude_deg,
					m.pos.longitude_deg,
					m.pos.altitude_m + offset_height,
					0,
				)

				#If close to mission
				if (dist(drone.pos, m.pos) <= 2):
					#Next mission
					drone.mission += 1
					next_drone = [i for i in drones if i.rank == drone.rank-1][0]
					#If other drone already on this mission, follow it (avoid collisions)
					if (next_drone.mission == drone.mission):
						drone.state = state.Follow
					else: #Otherwise go to mission
						drone.state = state.FollowMission
					
					#If out of missions, die (i dont think this code should ever trigger)
					if (drone.mission == len(mission)):
						drone.state = State.End

			#The front drone simply travels to waypoints
			if drone.state == State.Travel:
				m = mission[drone.mission]
				#If at mission
				if (dist(drone.pos, m.pos) <= 2):
					#If should stop
					if (m.stop and not m.visited):
						drone.state = State.Wait
						m.visited = True
						prev_drone = [i for i in drones if i.rank == drone.rank+1][0]
						prev_drone.state = State.Travel

						#Set yourself as last, and wait here
						for i in drones:
							i.rank -= 1
						drone.rank = len(drones)
					else: #Otherwise next mission
						drone.mission += 1
						if (drone.mission == len(mission)):
							drone.state = State.End
				else: #Otherwise continue going to mission
					await drone.inst.action.goto_location(
						m.pos.latitude_deg,
						m.pos.longitude_deg,
						m.pos.altitude_m + offset_height,
						0,
					)

			#If waiting at stop
			if drone.state == State.Wait:
				m = mission[drone.mission]
				if (dist(drone.pos, get_ambulance_pos()) <= 30):
					await drone.inst.action.goto_location(
						m.pos.latitude_deg,
						m.pos.longitude_deg,
						m.pos.altitude_m,
						0,
					)
					print("Activate alert")
				#TODO: If ambulance is past, set state to Follow

		await asyncio.sleep(0.2)

	print("-- Landing")
	for drone in drones:
		await drone.action.land()

	print("-- Done")
	running = False

if __name__ == "__main__":
	asyncio.ensure_future(run())
	asyncio.get_event_loop().run_forever()