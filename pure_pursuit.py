import glob
import os
import sys

try:    #überprüft os für carla
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import numpy as np
import math
import ast

L = 2.875   #wheelbase
Kdd = 4.0   #faktor für look ahead
alpha_prev = 0
delta_prev = 0

client = carla.Client('localhost', 2000)
client.set_timeout(200)

world = client.load_world("Town10HD")   #loads map

world.set_weather(carla.WeatherParameters.ClearNoon)    #sets weather

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]

transform = carla.Transform()

transform.location.x = 106      #richtung rathaus
transform.location.y = 30       #weg von engen wolkenkratzern
transform.location.z = 6        #höhe

transform.rotation.yaw = 270    #0 richtung rathaus, 270 richtung enge wolkenkratzer
transform.rotation.pitch = 0
transform.rotation.roll = 0

vehicle = world.spawn_actor(vehicle_bp, transform)

spectator = world.get_spectator()
sp_transform = carla.Transform(transform.location + carla.Location(z=50, x=0, y=0),
                               carla.Rotation(yaw=0, pitch=0))
spectator.set_transform(sp_transform)

control = carla.VehicleControl()
#control.throttle = 0.35              #speed
vehicle.apply_control(control)

map = world.get_map()
waypoints = map.generate_waypoints(2.0)

# Vehicle Current Location
vehicle_loc = vehicle.get_location()
wp = map.get_waypoint(vehicle_loc, project_to_road=True,
                      lane_type=carla.LaneType.Driving)
waypoint_list = []
with open('waypoints.txt','r') as file:
    content = file.read()
    waypoint_list=ast.literal_eval(content)

waypoint_obj_list = waypoint_list
speed = []
with open('speed.txt','r') as file:
    content = file.read()
    speed = ast.literal_eval(content)


# print values
def display(disp=False):        #prints informations in colsole
    if disp:
        print("--" * 20)
        print("\nMin Index= ", min_index)
        print("Forward Vel= %.3f m/s" % vf)
        print("Lookahead Dist= %.2f m" % ld)
        print("Alpha= %.5f rad" % alpha)
        print("Delta= %.5f rad" % steer_angle)
        print("Error= %.3f m" % e)
        print("Ticks von noOWp %f" % t)


# Calculate Delta
def calc_steering_angle(alpha, ld):
    delta_prev = 0.001
    delta = math.atan2(2 * L * np.sin(alpha), ld)
    delta = np.fmax(np.fmin(delta, 1.0), -1.0)
    if math.isnan(delta):
        delta = delta_prev
    else:
        delta_prev = delta

    return delta


# Get target waypoint index
def get_target_wp_index(veh_location, waypoint_list):
    dxl, dyl = [], []
    for i in range(len(waypoint_list)):
        dx = abs(veh_location.x - waypoint_list[i][0])
        dxl.append(dx)
        dy = abs(veh_location.y - waypoint_list[i][1])
        dyl.append(dy)

    dist = np.hypot(dxl, dyl)
    idx = np.argmin(dist) + 4

    # take closest waypoint, else last wp
    if idx < len(waypoint_list):
        tx = waypoint_list[idx][0]
        ty = waypoint_list[idx][1]
    else:
        tx = waypoint_list[-1][0]
        ty = waypoint_list[-1][1]

    return idx, tx, ty, dist


def get_lookahead_dist(vf, idx, waypoint_list, dist):   #calculates look ahead disance
    ld = Kdd * vf
    #while ld > dist[idx] and (idx+1) < len(waypoint_list):
    #    idx += 1
    return ld


# Debug Helper
def draw(loc1, loc2=None, type=None):
    if type == "string":
        world.debug.draw_string(loc1, "X",
                                life_time=2000, persistent_lines=True)
    elif type == "line":
        world.debug.draw_line(loc1, loc2, thickness=0.8,
                              color=carla.Color(r=0, g=255, b=0),
                              life_time=0.5, persistent_lines=True)
    elif type == "string2":
        world.debug.draw_string(loc1, "X", color=carla.Color(r=0, g=255, b=0),
                                life_time=0.3, persistent_lines=True)


# Generate waypoints
noOfWp = 17189    #number of waypoints
#t = 0
#while t < noOfWp:
 #   wp_next = wp.next(2.0)
  #  if len(wp_next) > 1:
   #     wp = wp_next[1]
    #else:
     #   wp = wp_next[0]

    #waypoint_obj_list.append(wp)
    #waypoint_list.insert(t, (wp.transform.location.x, wp.transform.location.y))
    #draw(wp.transform.location, type="string")
    #t += 1

#PID Speed
class PIDController:
    def __init__(self, P, I, D, lim_min, lim_max):
        self.KP = P
        self.KI = I
        self.KD = D
        self.lim_min = lim_min
        self.lim_max = lim_max
        self.error_i = 0
        self.error_prev = 0

    def control(self, error, delta_time):
        error_d = (error - self.error_prev) / delta_time
        self.error_i += error
        self.error_prev = error
        result = self.KP * error + self.KI * self.error_i + self.KD * error_d
        return max(self.lim_min, min(result, self.lim_max))


def get_vehicle_speed(vehicle):
    # Sie müssen diese Implementierung mit CARLA spezifischen Befehlen zur Geschwindigkeitsmessung ersetzen
    v=vehicle.get_velocity()
    velocity =math.sqrt(v.x**2 + v.y**2 + v.z**2)
    return velocity


def steuerung(vehicle, controller, target_speed, delta_time):
    # Geschwindigkeit des Fahrzeugs ermitteln
    current_speed = get_vehicle_speed(vehicle)

    # Fehler berechnen
    error = target_speed - current_speed

    control = carla.VehicleControl()
    # Wenn der Fehler positiv ist: Throttle-Wert und Brake-Wert auf 0
    if error > 0:
        control.throttle = controller.control(error, delta_time)
        control.brake = 0
    # Wenn der Fehler negativ ist: Brake-Wert und Trottle-Wert auf 0
    else:
        control.brake = controller.control(-error, delta_time)
        control.throttle = 0
    vehicle.apply_control(control)

#Game Loop
t = 0
#while t < noOfWp:
veh_location = vehicle.get_location()
min_index, _, _, _ = get_target_wp_index(veh_location, waypoint_list)
while min_index < noOfWp:
    veh_transform = vehicle.get_transform()
    veh_location = vehicle.get_location()
    veh_vel = vehicle.get_velocity()
    vf = np.sqrt(veh_vel.x ** 2 + veh_vel.y ** 2)
    vf = np.fmax(np.fmin(vf, 2.5), 0.1)

    min_index, tx, ty, dist = get_target_wp_index(veh_location, waypoint_list)
    ld = get_lookahead_dist(vf, min_index, waypoint_list, dist)

    yaw = np.radians(veh_transform.rotation.yaw)
    alpha = math.atan2(ty - veh_location.y, tx - veh_location.x) - yaw
    # alpha = np.arccos((ex*np.cos(yaw)+ey*np.sin(yaw))/ld)

    if math.isnan(alpha):
        alpha = alpha_prev
    else:
        alpha_prev = alpha

    e = np.sin(alpha) * ld
    controller = PIDController(P=0.1,I=0.005,D=0.2,lim_min=0,lim_max=1)
    target_speed = speed[t]
    delta_time = 0.001
    steuerung(vehicle,controller,target_speed,delta_time)
    steer_angle = calc_steering_angle(alpha, ld)
    control.steer = steer_angle
    vehicle.apply_control(control)

    #draw(veh_location, waypoint_obj_list[min_index].transform.location, type="line")    #draws car travel line
    #if min_index != (noOfWp-1):
    #    draw(waypoint_obj_list[min_index].transform.location, type="string2")              #highlights next waypoint

    display(disp=True)

    #time.sleep(0.5)
    t += 1

#print(waypoint_list)
vehicle.destroy()
print("Task Done!")
