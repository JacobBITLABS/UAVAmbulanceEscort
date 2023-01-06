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
	Travel = 3 #Normal travel
	TravelWait = 4 #Is decending to waypoint
	Wait = 5 #Is at a waypoint
	End = 6 #At the end

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
mission = [
	Mission(0, Pos(47.3977419, 8.5455937, 10), False),
	Mission(1, Pos(47.397459473771, 8.546179442458623, 10), True),
	Mission(2, Pos(47.39707095615432, 8.54589240348472, 10), True),
	Mission(3, Pos(47.3970510263338, 8.546029192499857, 10), True),
	Mission(4, Pos(47.39741230621734, 8.546302785162757, 10), True)
]
running = True
offset_height = 5.0


default_height = 8.0 #in Meters
follow_distance = 2.0 #in Meters, this is the distance that the drone will remain away from Target while following it
#Direction relative to the Target
#Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
direction = Config.FollowDirection.BEHIND
responsiveness = 0.02

def get_ambulance_pos():
	return [47.3977419, 8.5455937]

def dist(a,b):
	return geopy.distance.geodesic((a.latitude_deg, a.longitude_deg), (b.latitude_deg, b.longitude_deg)).m

async def print_position(idx):
	global drones
	global running

	async for position in drones[idx].inst.telemetry.position():
		drones[idx].pos = Pos(position.latitude_deg, position.longitude_deg, position.relative_altitude_m)
		if (not running):
			break

async def print_velocity(idx):
	global drones
	global running

	async for odom in drones[idx].inst.telemetry.position_velocity_ned():
		drones[idx].speed = [odom.velocity.north_m_s, odom.velocity.east_m_s, odom.velocity.down_m_s]
		if (not running):
			break

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

	for k,drone in enumerate(drones):
		asyncio.ensure_future(print_position(k))
		asyncio.ensure_future(print_velocity(k))

	print("-- Arming")
	for drone in drones:
		await drone.inst.action.arm()

	conf = Config(default_height, follow_distance, direction, responsiveness)
	for drone in drones:
		await drone.follow_me.set_config(conf)

	print("-- Taking off")
	for drone in drones:
		await drone.inst.action.takeoff()

	await asyncio.sleep(20)

	while (running):
		for k,drone in enumerate(drones):
			if drone.state == State.Start:
				if (drone.idx == 0):
					m = mission[drone.mission]
					await drone.inst.action.goto_location(
						m.latitude_deg,
						m.longitude_deg,
						m.altitude_m + offset_height,
						0,
					)
					drone.state = State.Travel
				else:
					await drone.follow_me.start()
					drone.state = State.Follow
			if drone.state == State.Follow:
				next_drone = [i for i in drone if i.rank == drone.rank-1][0]
				target = TargetLocation(
					next_drone.latitude_deg, next_drone.longitude_deg, next_drone.altitude_m, 
					next_drone.speed[0], next_drone.speed[1], next_drone.speed[2]
				)
				await drone.follow_me.set_target_location(target)
			if drone.state == State.Travel:
				pass #TODO: Are we there, and is it a stop?

		await asyncio.sleep(0.2)

	#print("-- Fly")
	#for k,drone in enumerate(drones):
	#	pos = drone.pos
	#	await drone.inst.action.goto_location(
	#		pos.latitude_deg,
	#		pos.longitude_deg,
	#		pos.absolute_altitude_m + 1,
	#		0,
	#	)

	#TODO: Scheme
	#Follow a mission for the front
	#When a stop is hit, drop the front
	#Then the one behind becomes the new front
	#

	#await asyncio.sleep(60)

	print("-- Landing")
	for drone in drones:
		await drone.action.land()

	print("-- Done")
	running = False

if __name__ == "__main__":
	asyncio.ensure_future(run())
	asyncio.get_event_loop().run_forever()