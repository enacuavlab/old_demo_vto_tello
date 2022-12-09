#!/usr/bin/env python3
#---------------------------------------------------
import math
import time
import transformations as xf
import numpy as np

import PID

import rclpy
import sys
import yaml
import os
from rclpy.qos import QoSHistoryPolicy


from enum import Enum
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter
from uav_interfaces.msg import State, Action, FlightData, Info, Pid, ParamUpdates, PathForm, AttractingPoint, Mission, Order, Status
from builtin_interfaces.msg import Time

from std_msgs.msg import Empty, Int32, String
from geometry_msgs.msg import Twist, TransformStamped, PointStamped
from tf2_msgs.msg import TFMessage

from ament_index_python.packages import get_package_share_directory

import COMPUTE_IJ_SPM as COMPUTE
import STREAMLINE_SPM as STREAMLINE
import decimal

package_dir = get_package_share_directory('fish_sim')
path = os.path.join(package_dir,'config', 'flock_param_real.yaml')#real is to_test_out 'test_5_uav', 'test_param.yaml') #'flock_param_real.yaml')

#-------------Parameters----------------------------------------

with open(path) as yaml_file:
    parsed_yaml_file = yaml.load(yaml_file, Loader=yaml.FullLoader)

    navfish_bodylength = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_bodylength"]
    navfish_maxvelocity = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_maxvelocity"]
    navfish_velocity = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_velocity"]
    navfish_minvelocity = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_minvelocity"]
    navfish_mind2d = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_mind2d"]
    navfish_fluct = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_fluct"]

    navfish_ew1 = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_ew1"]
    navfish_ew2 = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_ew2"]
    navfish_alpha = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_alpha"]
    navfish_yw = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_yw"] 
    navfish_lw = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_lw"] 
    navfish_yatt = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_yatt"]
    navfish_latt = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_latt"]
    navfish_d0att = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_d0att"]
    navfish_yali = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_yali"]
    navfish_lali = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_lali"]
    navfish_d0ali = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_d0ali"]
    navfish_alt = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_alt"]
    navfish_walldistance = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_walldistance"]
    navfish_yacc = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_yacc"]
    navfish_lacc = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_lacc"]
    navfish_dv0 = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_dv0"]
    navfish_ew1_ob = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_ew1_ob"]
    navfish_ew2_ob = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_ew2_ob"]
    navfish_yob = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_yob"] # entre 0.5 et 2
    navfish_lob = parsed_yaml_file["Agent"]["ros__parameters"]["navfish_lob"] # entre 0.5 et 2
    navfish_y_perp= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_y_perp"] # entre 0.5 et 2
    navfish_y_para= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_y_para"] # entre 0.5 et 2
    navfish_y_z= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_y_z"] # entre 0.5 et 2
    navfish_a_z= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_a_z"] # entre 0.5 et 2
    navfish_zmax= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_zmax"] # entre 0.5 et 2
    navfish_zmin= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_zmin"] # entre 0.5 et 2
    navfish_dz0= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_dz0"] # entre 0.5 et 2
    navfish_alpha_z= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_alpha_z"] # entre 0.5 et 2
    navfish_L_z_2= parsed_yaml_file["Agent"]["ros__parameters"]["navfish_L_z_2"] # entre 0.5 et 2
    


#-------------Primary Function------------------------------------------
def ros_time_to_time(ros_time: Time) -> float:
    return ros_time.sec*1000 + ros_time.nanosec / 1000000

def drange(x, y, jump):
  while x < y:
    yield float(x)
    x = decimal.Decimal(x) + decimal.Decimal(jump)

class Building():
    def __init__(self,index_b, type_obst,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
        self.vertices = np.array(vertices)
        self.type_obst = type_obst
        i_vertice = 0
        self.index_b = index_b
        x_0,y_0 = 0, 0
        for vertice in self.vertices:
            i_vertice+=1
            x_0+=float(vertice[0])
            y_0+=float(vertice[1])
        self.center = [x_0/i_vertice, y_0/i_vertice]
        self.radius = max([math.sqrt((vertice[0]-self.center[0])**2 + (vertice[1]-self.center[1])**2) for vertice in self.vertices])
class TrajectoryHandler(object):

    def __init__(self):

        # set up the variables of interest
        self._rel_times = np.array([])  # list of the relative times for the trajectory in decimal seconds
        self._positions = np.array([])  # list of the positions for the trajectory in
        self.s = None
        self.ke = None
        self.params = None
        self.type = None
        self.d_corr = None
        self.track_error = None
        self.progress = None
        self.yaw_traj = 0.0
        self.yaw_traj_panels = 0.0

        self.source = None
        self.V_inf = 0.5
        self.alpha_inf = None
        #self.buildings = [Building([[4.62, 5.52, 1.2], [0.79, 0.97, 1.2], [5.65, 4.4, 1.2], [0.08, 2.32, 1.2], [6., 2.92, 1.2]])]
        self.buildings = [Building(1,'obstacle',[[5.0, 5.0, 1.2], [6.0, 5.0, 1.2], [6.0, 6.0, 1.2], [5.0,6.0, 1.2]])]
                            
        self.buildings = [#Building(1,'obstacle',[[3.0, 2.0, 1.2], [2.75, 1.567, 1.2], [2.25, 1.567, 1.2], [2.0, 2.0, 1.2], [2.25, 2.433, 1.2], [2.75, 2.433, 1.2]]),
                            Building(2,'obstacle',[[1.0, 3.0, 1.5], [0.75, 2.567, 1.5], [0.25, 2.567, 1.5], [0.0, 3.0, 1.5], [0.25, 3.433, 1.5], [0.75, 3.433, 1.5]]),
                            Building(3,'obstacle',[[1.0, 0.5, 2], [0.75, 0.067, 2], [0.25, 0.067, 2], [0.0, 0.5, 2], [0.25, 0.933, 2], [0.75, 0.933, 2]]),   
                            #Building(4,'obstacle',[[-2.65, 1.5, 1.5], [-3.0, 1.15, 1.5], [-3.35, 1.5, 1.5], [-3.0, 1.85, 1.5]]), 
                            #Building(5, 'obstacle',[[-2.65, -1.5, 1.5], [-3.0, -1.85, 1.5], [-3.35, -1.5, 1.5], [-3.0, -1.15, 1.5]]),  
                            Building(6,'obstacle',[[-1.15, -0.2, 1.5], [-1.5, -0.55, 1.5], [-1.85, -0.2, 1.5], [-1.5, 0.15, 1.5]]), 
                            #Building(7,'obstacle',[[1.0, -2.0, 2], [0.75, -2.433, 2], [0.25, -2.433, 2], [0.0, -2.0, 2], [0.25, -1.567, 2], [0.75, -1.567, 2]]),
                            #Building(8,'obstacle', [[2.511232215308076, 7.95814036687515, 1.0], [1.2404186468869003, 9.228953935296325, 1.0], [-0.030394921534275143, 7.95814036687515, 1.0], [1.2404186468869, 6.687326798453975, 1.0]]),
                            #Building(9,'obstacle', [[-4.186530021394977, -6.122157473007361, 2.25], [-4.446146410401916, -5.495388065652479, 2.25], [-5.072915817756798, -5.235771676645539, 2.25], [-5.69968522511168, -5.495388065652479, 2.25], [-5.95930161411862, -6.122157473007361, 2.25], [-5.699685225111681, -6.748926880362243, 2.25], [-5.072915817756798, -7.0085432693691825, 2.25], [-4.446146410401917, -6.7489268803622435, 2.25]]),
                            #Building(10,'obstacle', [[-5.222778140967321, 0.4276621001287385, 2.5], [-5.936391716869527, 1.4098669242033168, 2.5], [-7.091042737512647, 1.034698065320939, 2.5], [-7.091042737512647, -0.17937386506346187, 2.5], [-5.936391716869527, -0.5545427239458399, 2.5]]),
                            #Building(11,'obstacle', [[10.278140144555621, -4.148350198141244, 2.0], [9.382951637303346, -2.597838221228561, 2.0], [7.592574622798794, -2.5978382212285607, 2.0], [6.697386115546517, -4.148350198141244, 2.0], [7.592574622798793, -5.698862175053926, 2.0], [9.382951637303346, -5.698862175053926, 2.0]]),
                            #Building(12,'obstacle', [[3.3680249207960857, -5.911803894264205, 1.0], [3.146680252412709, -5.452176954348914, 1.0], [2.6493232971636544, -5.338658474870764, 1.0], [2.2504739865844394, -5.656730186231781, 1.0], [2.2504739865844394, -6.166877602296629, 1.0], [2.649323297163655, -6.484949313657646, 1.0], [3.146680252412709, -6.371430834179495, 1.0]]),
                            #Building(13,'obstacle', [[6.9650375330734775, 7.551036173798959, 3.0], [6.61042494978808, 8.407146681754725, 3.0], [5.754314441832314, 8.761759265040123, 3.0], [4.898203933876548, 8.407146681754725, 3.0], [4.54359135059115, 7.551036173798959, 3.0], [4.898203933876547, 6.6949256658431935, 3.0], [5.754314441832314, 6.340313082557795, 3.0], [6.610424949788079, 6.694925665843193, 3.0]]),
                            #Building(14,'obstacle', [[-5.649040306695463, 5.605150311414378, 1.0], [-6.770057349977365, 6.72616735469628, 1.0], [-7.891074393259267, 5.605150311414378, 1.0], [-6.770057349977365, 4.484133268132476, 1.0]]),
                            #Building(15,'obstacle', [[1.6844613126479104, -6.965900655669251, 2.5], [1.0518275732941587, -5.870146876526266, 2.5], [-0.21343990541334523, -5.870146876526266, 2.5], [-0.8460736447670976, -6.965900655669251, 2.5], [-0.21343990541334612, -8.061654434812235, 2.5], [1.0518275732941587, -8.061654434812235, 2.5]]),
                            Building(16,'obstacle', [[3.0, -0.5, 1.2], [2.75, -0.933, 1.2], [2.25, -0.933, 1.2], [2.0, -0.5, 1.2], [2.25, -0.067, 1.2], [2.75, -0.067, 1.2]])]
        self.buildings = [Building(16,'obstacle', [[20., 20., 1.2], [21., 20., 1.2], [21., 21., 1.2], [20., 21., 1.2]])]
        self.buildings = []
                 
        """new_buildings = []
        XY_upper = []
        for theta_value in list(drange(0.197396,math.pi/2,'0.05')):
            XY_upper.append([5*math.cos(theta_value), 5*math.sin(theta_value), 0.])
        for theta_value in list(drange(math.pi/2, math.pi - 0.197396,'0.05')):
            XY_upper.append([20 + 5*math.cos(theta_value), 5*math.sin(theta_value), 0.])
        self.buildings.append(Building(16,'boundary', XY_upper))

        XY_lower = []
        for theta_value in list(drange(-math.pi/2, -0.197396,'0.05')):
            XY_lower.append([5*math.cos(theta_value), 5*math.sin(theta_value), 0.])
        for theta_value in list(drange(math.pi + 0.197396,3*math.pi/2,'0.05')):
            XY_lower.append([20 + 5*math.cos(theta_value), 5*math.sin(theta_value), 0.])
        self.buildings.append(Building(17,'boundary', XY_lower))

        for i_b, building in enumerate(self.buildings[:-2]):
            XYZ = [[20+vertice[0], vertice[1], vertice[2]] for vertice in building.vertices]
            new_buildings.append(Building(18+i_b, 'obstacle', XYZ))
        for building in new_buildings:
            self.buildings.append(building) """
        self.lam_array=np.array([]) #Panel strengths

        self.zone_points= None
        self.zone_id = None
        self.two_closest_points = None

    def update_panels(self, x, y):
        self.lam_array=np.array([]) #Panel strengths
        for building in self.buildings:
            XB = [vertice[0] for vertice in building.vertices]+[building.vertices[0][0]]
            YB = [vertice[1] for vertice in building.vertices]+[building.vertices[0][1]]
            numPan = len(XB)-1
            edge = np.zeros(numPan)
            for i in range(numPan):
                edge[i] = (XB[i+1]-XB[i])*(YB[i+1]+YB[i])
            sumEdge = np.sum(edge) 
            if (sumEdge < 0):
                XB = np.flipud(XB)
                YB = np.flipud(YB)
            XC  = np.zeros(numPan)
            YC  = np.zeros(numPan)
            S   = np.zeros(numPan)
            phi = np.zeros(numPan)

            for i in range(numPan):
                XC[i]   = 0.5*(XB[i]+XB[i+1])                                               # X-value of control point
                YC[i]   = 0.5*(YB[i]+YB[i+1])                                               # Y-value of control point
                dx      = XB[i+1]-XB[i]                                                     # Change in X between boundary points
                dy      = YB[i+1]-YB[i]                                                     # Change in Y between boundary points
                S[i]    = (dx**2 + dy**2)**0.5                                              # Length of the panel
                phi[i]  = math.atan2(dy,dx)                                                 # Angle of panel (positive X-axis to inside face)
                if (phi[i] < 0):                                                            # Make all panel angles positive [rad]
                    phi[i] = phi[i] + 2*np.pi
            self.dist_to_source = math.sqrt((self.source[0]-x)**2 + (self.source[1]-y)**2)
            AoAR_ex = math.atan2(self.source[1]-YC[i],self.source[0]-XC[i])
            delta                = phi + (np.pi/2)                                          # Angle of panel normal [rad]
            beta                 = delta - AoAR_ex                                             # Angle of panel normal and AoA [rad]
            beta[beta > 2*np.pi] = beta[beta > 2*np.pi] - 2*np.pi                           # Make all panel angles between 0 and 2pi [rad]
            I, J = COMPUTE.COMPUTE_IJ_SPM(XC,YC,XB,YB,phi,S)
            A = I + np.pi*np.eye(numPan,numPan)
            b = np.zeros(numPan)
            for i in range(numPan):
                b[i] = -self.V_inf*2*np.pi*np.cos(beta[i])
            lam = np.dot(np.linalg.inv(A), b)
            self.lam_array = np.concatenate((self.lam_array, lam))

    def update_track(self, x, y, neighbors):
        nx = - math.sin(self.params[2]) #self.params = [a, b, alpha, length]
        ny = math.cos(self.params[2])
        tx = self.s*ny
        ty = - self.s*nx

        #print("Normal vector is now ", (nx, ny), " and Tangential vector is ", (tx,ty))
        self.track_error = (x-self.params[0])*nx + (y -self.params[1])*ny

        self.progress = 0.5 + ((x-self.params[0])*tx + (y -self.params[1])*ty)*2/self.params[3]
        #print ("Went ", x-self.params[0], "meters along the tx axis, ", y -self.params[1], "along the ty axis, to travel a distance of ", self.params[3], 'meters')


        if self.track_error > self.d_corr:
            self.track_error -= self.d_corr
        elif self.track_error < - self.d_corr:
            self.track_error += self.d_corr
        
        U = tx - self.ke * self.track_error * nx
        V = ty - self.ke * self.track_error * ny

        norm = math.sqrt(U**2 + V**2)

        U_norm, V_norm = U/norm, V/norm
        #print(" projeté sur x: ", U_norm, " et projeté sur y: ", V_norm )
        #print("Pour rappel, les params sont : ", self.params)
        #print('Going towards angle atan2(VU): ', math.atan2(V_norm, U_norm))
        self.yaw_traj = math.atan2(V_norm, U_norm)

    def update_track_source(self, x, y):
        AoAR = math.atan2(self.source[1]-y, self.source[0]-x)
        Vx = self.V_inf*np.cos(AoAR)
        Vy = self.V_inf*np.sin(AoAR)
        i_start_lam=0
        self.progress_source = math.sqrt((self.source[0]-x)**2 + (self.source[1]-y)**2)
        for building in self.buildings:
            XB = [vertice[0] for vertice in building.vertices]+[building.vertices[0][0]]
            YB = [vertice[1] for vertice in building.vertices]+[building.vertices[0][1]]
            numPan = len(XB)-1
            S   = np.zeros(numPan)
            phi = np.zeros(numPan)
            for i in range(numPan):                                              # Y-value of control point
                dx      = XB[i+1]-XB[i]                                                     # Change in X between boundary points
                dy      = YB[i+1]-YB[i]                                                     # Change in Y between boundary points
                S[i]    = (dx**2 + dy**2)**0.5                                              # Length of the panel
                phi[i]  = math.atan2(dy,dx)                                                 # Angle of panel (positive X-axis to inside face)
                if (phi[i] < 0):                                                            # Make all panel angles positive [rad]
                    phi[i] = phi[i] + 2*np.pi
            Mx, My = STREAMLINE.STREAMLINE_SPM(x,y,XB,YB,phi,S)
            Vx += sum(self.lam_array[i_start_lam:i_start_lam+numPan]*Mx/(2*np.pi))
            Vy += sum(self.lam_array[i_start_lam:i_start_lam+numPan]*My/(2*np.pi))
        self.yaw_traj_panels = math.atan2(Vy, Vx)

    def which_points(self, x, y):
        self.two_closest_points = sorted([math.sqrt((self.zone_points[i]-x)**2+(self.zone_points[i+1]-y)**2) for i in range(0,len(self.zone_points),2)])[:2]

    def go_back(self, x, y):
        two_closest_points = self.two_closest_points
        x1,y1,x2,y2 = two_closest_points
        x_mid,y_mid= (x1+x2)/2, (y1+y2)/2
        alpha = math.atan2(y_mid-y,x_mid-x)
        nx = - math.sin(alpha) #self.params = [a, b, alpha, length]
        ny = math.cos(alpha)
        tx = self.s*ny
        ty = - self.s*nx

        #print("Normal vector is now ", (nx, ny), " and Tangential vector is ", (tx,ty))
        self.track_error = (x-self.params[0])*nx + (y -self.params[1])*ny

        self.progress = 0.5 + ((x-self.params[0])*tx + (y -self.params[1])*ty)*2/self.params[3]
        #print ("Went ", x-self.params[0], "meters along the tx axis, ", y -self.params[1], "along the ty axis, to travel a distance of ", self.params[3], 'meters')


        if self.track_error > self.d_corr:
            self.track_error -= self.d_corr
        elif self.track_error < - self.d_corr:
            self.track_error += self.d_corr
        
        U = tx - self.ke * self.track_error * nx
        V = ty - self.ke * self.track_error * ny

        norm = math.sqrt(U**2 + V**2)

        U_norm, V_norm = U/norm, V/norm
        #print(" projeté sur x: ", U_norm, " et projeté sur y: ", V_norm )
        #print("Pour rappel, les params sont : ", self.params)
        #print('Going towards angle atan2(VU): ', math.atan2(V_norm, U_norm))
        self.yaw_traj = math.atan2(V_norm, U_norm)



#-------------Controller----------------------------------------


class Controller():
    def __init__(self):
        print("Initiated the Controller Class!")
        self.i = 0

        self._kp_alt = 1.0
        self.hdot_max = 0.6

        self._kp_yaw = 6.1#2.0 #6.1  # gain for yaw error
        self._ki_yaw = 0.0#0.01 #0.0
        self._kd_yaw = 4.8#0.6 #4.8

        self.PID = PID.PID(self._kp_yaw, self._ki_yaw, self._kd_yaw, True)
        self.PID.setSampleTime(0.1)
        self._yawdot_max = 1.2 # the maximum yaw rate
        self.yaw_cmd_test = -1.57 ##################################

    def yaw_control(self, yaw_cmd, yaw, yawdot_target=0.0):
        #self.PID.SetPoint = self.yaw_cmd_test ###############################
        self.PID.SetPoint = yaw_cmd
        self.PID.update(yaw)
        self.yaw=yaw
        #self.yaw_cmd=yaw_cmd
        yawdot_cmd = self.PID.output #, ret_kp, ret_ki, ret_kd = self.PID.output
        yawdot_cmd_clipped = np.clip(yawdot_cmd, -self._yawdot_max, self._yawdot_max)
        #if self.i%2== 0:
        #    print("yaw_cmd", self.yaw_cmd_test, "yawdot_cmd ", yawdot_cmd, " ret_kp ", ret_kp, " ret_ki", ret_ki, " and ret_kd", ret_kd)
        return yawdot_cmd_clipped
    
    def altitude_control(self, alt_cmd, alt, hdot_target=0.0):

        alt_error = alt_cmd - alt
        hdot_cmd = self._kp_alt* alt_error + hdot_target
        hdot_cmd = np.clip(hdot_cmd, -self.hdot_max, self.hdot_max)
        return hdot_cmd


#-------------Agent----------------------------------------
class Agent(Node):
    class States(Enum):
        INIT = 0
        WAITING = 1
        LAUNCHING = 2
        FLOCKING = 3
        LANDING = 4
        DONE = 5
        NAVIGATING = 6
        HUNTING = 7
        EXPLORING = 8
        BACKUP = 9
        HOLDING = 10
        RETURNING_HOME = 11
        FLIPPING = 12

    def __init__(self, id_uav, nb_uav, z_start):
        node_name = 'agent'+str(id_uav)
        super().__init__(node_name)
        self.z_start = z_start

        self.has_flipped = False

        self._controller = Controller()
        self.lag_limit_milisec = 1500
        self._state = self.States.INIT
        #self.create_timer(10, self.state_mode_callback)
        self.create_timer(0.1, self._process_interactions)
        self.time0_log=time.time()
        self.index = id_uav
        self.i = 0
        self.nb_uav = nb_uav
        self.random_iter = 0
        self.random_iter_done = False
        self.name = 'drone' + str(id_uav)
        self.arena_pose = [0.0, 0.0]
        self.LEAVE_ARENA = False
        self.first_waiting=True
        self.flip_call_count=0

        self.dt = 0.1
        #self.last_tf_time = 0
        self.last_sent_time_ns = self.get_clock().now().nanoseconds
        self.last_msg_time = self.last_sent_time_ns

        self._already_flying = False  # don't launch and land
        self._prev_state = None
        self.params = {
            'navfish_bodylength': navfish_bodylength,
            'navfish_maxvelocity': navfish_maxvelocity,
            'navfish_velocity': navfish_velocity,
            'navfish_minvelocity': navfish_minvelocity,
            'navfish_mind2d': navfish_mind2d,
            'navfish_fluct': navfish_fluct,
            'navfish_ew1': navfish_ew1,
            'navfish_ew2': navfish_ew2,
            'navfish_ew1_ob': navfish_ew1_ob,
            'navfish_ew2_ob': navfish_ew2_ob,
            'navfish_alpha': navfish_alpha,
            'navfish_yw': navfish_yw,
            'navfish_lw': navfish_lw,
            'navfish_yatt': navfish_yatt,
            'navfish_latt': navfish_latt,
            'navfish_d0att': navfish_d0att,
            'navfish_yali': navfish_yali,
            'navfish_lali': navfish_lali,
            'navfish_d0ali': navfish_d0ali,
            'navfish_alt': navfish_alt,
            'navfish_walldistance': navfish_walldistance,
            'navfish_yacc': navfish_yacc,
            'navfish_lacc': navfish_lacc,
            'navfish_dv0': navfish_dv0,
            'navfish_yob': navfish_yob,
            'navfish_lob': navfish_lob,
            'navfish_y_perp': navfish_y_perp,
            'navfish_y_para': navfish_y_para,
            'navfish_y_z': navfish_y_z,
            'navfish_a_z': navfish_a_z,
            'navfish_zmax': navfish_zmax,
            'navfish_zmin': navfish_zmin,
            'navfish_dz0': navfish_dz0,
            'navfish_alpha_z': navfish_alpha_z,
            'navfish_L_z_2': navfish_L_z_2
        }
        self._drone_state = np.zeros(4)
        self._prev_pos_state = np.zeros(4)
        self._other_drone_state = [np.zeros(4) for i in range(20)]

        #self.get_logger().info("Params at launch are: "+str(self.params))
        #Local params
        self.velocity = self.params['navfish_minvelocity']
        self._traj_handler = TrajectoryHandler()
        self.used_pose=False

        # ROS publishers
        self._cmd_vel_pub = self.create_publisher(Twist, self.name + '/cmd_vel', 10)
        self._action_pub = self.create_publisher(Action, self.name + '/action', 1)
        self.order_pub_ = self.create_publisher(Order, '/order', 1)
        self.status_sub = self.create_subscription(Status, self.name + '/status', self.status_callback, 10)

        

        #self._other_pub = self.create_publisher(State, self.name + '/comm_pose', 1)
        self._info_pub = self.create_publisher(Info, self.name + '/info', 1)
        self._nav_ok_pub = self.create_publisher(Int32, '/nav_ok', 1)

        # ROS subscriptions
        #self._state_sub = self.create_subscription(State, '/drone' + str(self.index) + '/state', self._ros_state_callback, 1)
        self._tf_sub = self.create_subscription(TFMessage, '/tf', self._ros_tf_callback, 1)

        self._flight_data_sub = self.create_subscription(FlightData, 'flight_data', self._flight_data_callback, 1 )
        self._start_sub = self.create_subscription(Empty, '/start_mission', self._ros_start_callback, 1)
        self._stop_sub = self.create_subscription(Empty, '/stop_mission', self._ros_stop_callback, 1)
        #self._other_state_sub = [self.create_subscription(State, 'drone' +str(i)+ '/comm_pose', self._ros_other_state_callback, 1) for i in [x+1 for x in range(nb_uav) if x!=self.index-1]]
        self._other_state_active_i = []
        self._PID_sub = self.create_subscription(Pid, 'drone' +str(self.index)+ '/pid', self._pid_callback, 1)
        self._param_updates_sub = self.create_subscription(ParamUpdates, '/param_updates', self._param_updates_callback, 1)
        self.kill_sub = self.create_subscription(Empty, 'kill', self.kill_callback, 10)
        self._nav_sub = self.create_subscription(PathForm, '/nav', self._nav_callback, 1)
        self._mission_sub = self.create_subscription(Mission, '/mission', self._mission_callback, 1)
        self._att_point_sub = self.create_subscription(AttractingPoint, '/att_point', self._att_point_callback, 1)
        self._clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self._clicked_point_callback, 1)
        self.buttons_sub = self.create_subscription(String, '/buttons' , self.buttons_callback, 5)
        
        self.first_launch_msg = True
        self.get_logger().info(node_name + ' init complete.. write an Empty message on the topic /start_mission')
        
        # ROS parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('navfish_bodylength', navfish_bodylength),
                ('navfish_maxvelocity', navfish_maxvelocity),
                ('navfish_velocity', navfish_velocity),
                ('navfish_minvelocity', navfish_minvelocity),
                ('navfish_mind2d', navfish_mind2d),
                ('navfish_fluct', navfish_fluct),
                ('navfish_ew1', navfish_ew1),
                ('navfish_ew2', navfish_ew2),
                ('navfish_ew1_ob', navfish_ew1_ob),
                ('navfish_ew2_ob', navfish_ew2_ob),
                ('navfish_alpha', navfish_alpha),
                ('navfish_yw', navfish_yw),
                ('navfish_lw', navfish_lw),
                ('navfish_yatt', navfish_yatt),
                ('navfish_latt', navfish_latt),
                ('navfish_d0att', navfish_d0att),
                ('navfish_yali', navfish_yali),
                ('navfish_lali', navfish_lali),
                ('navfish_d0ali', navfish_d0ali),
                ('navfish_alt', navfish_alt),
                ('navfish_walldistance', navfish_walldistance),
                ('navfish_yacc', navfish_yacc),
                ('navfish_lacc', navfish_lacc),
                ('navfish_dv0', navfish_dv0),
                ('navfish_yob', navfish_yob),
                ('navfish_lob', navfish_lob),
                ('navfish_y_perp', navfish_y_perp),
                ('navfish_y_para', navfish_y_para),
                ('navfish_y_z', navfish_y_z),
                ('navfish_a_z', navfish_a_z),
                ('navfish_zmax', navfish_zmax),
                ('navfish_zmin', navfish_zmin),
                ('navfish_dz0', navfish_dz0),
                ('navfish_alpha_z', navfish_alpha_z),
                ('navfish_L_z_2', navfish_L_z_2)
            ]
        )
        self.i_log=0

    def state_mode_callback(self):
        self.get_logger().info("self.States is "+str(self._state))

    def start(self):
        self.get_logger().info('starting mission')
        if self._state == self.States.INIT:
            self._state = self.States.WAITING
        self._launch(self._drone_state)
    
    def buttons_callback(self,msg):
        if msg.data == 'start':
            self.start()
        if msg.data == 'stop':
            self.stop()
        if msg.data == 'takeoff':
            self._call_takeoff()
        if msg.data == 'land':
            self._call_land()

    def stop(self):
        if self._state != self.States.LANDING and self._state != self.States.FLIPPING:
            self.get_logger().info("Stopping mission..")
            self._state = self.States.RETURNING_HOME
        """if self._state == self.States.LAUNCHING or \
                self._state == self.States.FLOCKING:
            self._call_cmd(np.zeros(4))
        self._land()"""
        
    def _param_updates_callback(self, msg):
        if self.index in list(msg.indexes) or list(msg.indexes) == []:
            new_param = Parameter(msg.name, Parameter.Type.DOUBLE, msg.value)
            self.set_parameters([new_param])
            self.params[msg.name] = msg.value
        if msg.name != 'navfish_alt':
            self.get_logger().info("Changed "+ msg.name + " into: " + str(self.params[msg.name]))
    
    def status_callback(self, msg):
        id_uav = msg.index
        if msg.status[:4] == 'flip':
            self.has_flipped = True
            self._state = self.States.LANDING
    
    def _pid_callback(self, msg):
        self._controller.PID.setKp(msg.kp)
        self._controller.PID.setKi(msg.ki)
        self._controller.PID.setKd(msg.kd)
        self.get_logger().info("PID: Kp " + str(self._controller.PID.Kp) + ", Ki: " + str(self._controller.PID.Ki) + ", Kd: " + str(self._controller.PID.Kd))

    def _ros_tf_callback(self, msg):
        for ts in msg.transforms:
            #if ts.header.stamp.sec + ts.header.stamp.nanosec/1000000000 != self.last_tf_time:
                #self.dt = ts.header.stamp.sec + ts.header.stamp.nanosec/1000000000 - self.last_tf_time
                #self.last_tf_time= ts.header.stamp.sec + ts.header.stamp.nanosec/1000000000
            #time_now = time.time()
            #self.dt = time_now - self.last_tf_time
            #self.last_tf_time= time_now
            #self.get_logger().info("Self.dt is now: " +str(self.dt) )
            if ts.child_frame_id[:5] == 'drone':
                id_uav = int(ts.child_frame_id[-1])
                #self.get_logger().info(str(ts.child_frame_id))
                #self.get_logger().info('Received a TF message, and its id is '+str(id_uav))
                if self.index == id_uav:
                    self._ros_state_callback(ts)
                    self.used_pose=False
                else:
                    self._ros_other_state(ts, id_uav)
        

        '''ros_stamp = Time()
            ros_stamp.sec = int(stamp//1)
            ros_stamp.nanosec = int(stamp%1)
            msg.header.stamp = ros_stamp
            msg.header.frame_id = self._default_parent_frame
            msg.child_frame_id = 'drone'+str(ac_id-59) #Tello IDs start at 60

            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]

            msg.transform.rotation.x = quat[0]
            msg.transform.rotation.y = quat[1]
            msg.transform.rotation.z = quat[2]
            msg.transform.rotation.w = quat[3]'''

    def _ros_state_callback(self, msg): #RECEIVING A TF MESSAGE, NOT A STATE
        _, _, yaw = xf.euler_from_quaternion([msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
        yaw = yaw
        #self.get_logger().info('my yaw is '+str(yaw))
        if yaw > np.pi:
            yaw -= 2*np.pi
        if yaw < -np.pi:
            yaw += 2*np.pi
        state = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, yaw])
        #self.get_logger().info('Yaw is: '+str(yaw))
        self._new_drone_state(ros_time_to_time(msg.header.stamp), state)
        if self._state == self.States.NAVIGATING:
            self._traj_handler.update_track(state[0], state[1], self._other_state_active_i)
            if self.i < 20:
                self.i +=1
            else:
                self.i=0
                self.get_logger().info('Trajectory progress is about '+str(int(self._traj_handler.progress*100))+' % !! ')
                #self.get_logger().info('Tracking_error is about '+str(self._traj_handler.track_error)+' !! ')
                if self._traj_handler.progress > 0.95:
                    self.get_logger().info('Should now have arrived at the end of the line, but not necessarily in the corridor')
                if self._traj_handler.progress > 0.95:
                    self._state = self.States.FLOCKING
                    self.get_logger().info("Changed state to FLOCKING (after nav)")
                    self.arena_pose = [self._traj_handler.params[0] + math.cos(self._traj_handler.params[2] * self._traj_handler.params[3]/2),\
                                        self._traj_handler.params[1] + math.sin(self._traj_handler.params[2] * self._traj_handler.params[3]/2)]
                    self._nav_ok_pub.publish(Int32(data=self.index))

        if self._state == self.States.HUNTING:
            self._traj_handler.update_track_source(state[0], state[1])
            if self.i < 100:
                self.i +=1
            else:
                self.i=0
                self.get_logger().info('Hunting.. Distance from SOURCE is about '+str(self._traj_handler.progress_source)+' m !! ')
                if self._traj_handler.progress_source <0.5 :
                    self.needs_to_stop = True
                    self.get_logger().info("Changed state to FLOCKING (after navigating towards the source)")
                    self._nav_ok_pub.publish(Int32(data=self.index))
                    self._state = self.States.WAITING
                    self.LEAVE_ARENA=False
                    self.arena_pose = [0., 0.]
    
    def _ros_other_state(self, msg, index_other):
        other_index = index_other
        if other_index < self.index:
            if max(2*self.params['navfish_lali'],2*self.params['navfish_latt'], 2*self.params['navfish_lacc'])**2 > (msg.transform.translation.x-self._drone_state[0])**2 + (msg.transform.translation.y-self._drone_state[1])**2:
                if other_index not in self._other_state_active_i:
                    self._other_state_active_i.append(other_index)
                _,_,other_yaw = xf.euler_from_quaternion([msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
                other_yaw = other_yaw# + np.pi/2
                #self.get_logger().info("other yaw is "+str(other_yaw))
                if other_yaw > np.pi:
                    other_yaw -= 2*np.pi
                if other_yaw < -np.pi:
                    other_yaw += 2*np.pi
                try:
                    self._other_drone_state[other_index-1] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, other_yaw])
                except IndexError:
                    self.get_logger().info('Received an index not supposed to be considered')
                    pass
        else:
            if 2*max(2.5*self.params['navfish_lali'],2.5*self.params['navfish_latt'])**2 > (msg.transform.translation.x-self._drone_state[0])**2 * (msg.transform.translation.y-self._drone_state[1])**2:
                if other_index not in self._other_state_active_i:
                    self._other_state_active_i.append(other_index)
                _,_,other_yaw = xf.euler_from_quaternion([msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z])
                other_yaw = other_yaw# + np.pi/2
                #self.get_logger().info("other yaw is "+str(other_yaw))
                if other_yaw > np.pi:
                    other_yaw -= 2*np.pi
                if other_yaw < -np.pi:
                    other_yaw += 2*np.pi
                try:
                    self._other_drone_state[other_index-2] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, other_yaw])
                except IndexError:
                    self.get_logger().info('Received an index not supposed to be considered')
                    pass

    def kill_callback(self, msg):
        self._land()
        time.sleep(5)
        self.destroy_node()
        exit()

    def _nav_callback(self, msg):
        if self.index in list(msg.indexes) or list(msg.indexes) == []:    
            self._traj_handler.s, self._traj_handler.ke = msg.s, msg.ke
            self._traj_handler.d_corr = msg.d_corr
            if msg.type =='line':
                self._traj_handler.type = msg.type
                self._traj_handler.params = msg.params
                self.get_logger().info("Receive a 'Line' msg")
            else:
                self.get_logger().info("Didn't receive a 'Line' msg")
            if self._state == self.States.FLOCKING:
                self._state = self.States.NAVIGATING
                self.get_logger().info("Changed state to NAVIGATIING (after flocking)")
                self.get_logger().info("Now NAVIGATING")

    def _mission_callback(self, msg):
        if self.index in list(msg.indexes) or list(msg.indexes) == []:    
            if msg.state == 'init':
                self.get_logger().info("Recevied the list of point regarding my zone, sending them to traj_handler")
                self._traj_handler.zone_points = msg.points
                self._traj_handler.zone_id = msg.zone
            elif msg.state == 'out':
                self.get_logger().info("I'm apparently out of the zone, going back into it (TrajectoryHandler.go_back() function)")
                self.prev_state = self._state
                self._state = self.States.BACKUP
                self.get_logger().info("Changed state to BACKUP (after nav)")
                self._traj_handler.go_back()
                #Go back analysing every points
                #add a funcxtion go back to traj
            elif msg.state == 'out':
                self.get_logger().info("I'm apparently back in the zone, going back to my previous state, which was"+str(self._prev_state))
                self._state = self._prev_state
                self.get_logger().info("Changed state to self.prev_state (after mission nav)")

    
    def _att_point_callback(self, msg):
        self.get_logger().info("Got an att_point message")
        if self.index in list(msg.indexes) or list(msg.indexes) == []:
            self.get_logger().info("I am concerned !!!")
            self.get_logger().info('Got an attracting point to go to !! Going now like a corridor going towards it')
            self._traj_handler.source = [msg.x, msg.y]
            self._traj_handler.V_inf = 0.5
            x_drone = self._drone_state[0]
            y_drone = self._drone_state[1]
            self._traj_handler.alpha_inf =  math.atan2(msg.y - y_drone, msg.x - x_drone)
            if self._state == self.States.FLOCKING or self._state == self.States.NAVIGATING or self._state == self.States.WAITING:
                self.needs_to_stop = False
                self._state = self.States.HUNTING
                self.get_logger().info("Now HUNTING")
        else:
            self.get_logger().info("I am not concerned.. the list of indexes is "+str(list(msg.indexes))+", and I am index: "+str(self.index))
        self._traj_handler.update_panels(self._drone_state[0], self._drone_state[1])
            
    def _clicked_point_callback(self, msg):
        self.get_logger().info("Got an clicked_point message!!")
        self._traj_handler.source = [msg.point.x, msg.point.y]
        if math.sqrt((msg.point.x-self.arena_pose[0])**2+(msg.point.y-self.arena_pose[1])**2)> self.params['navfish_walldistance']:
            self.LEAVE_ARENA=False
            self._state = self.States.FLOCKING
            return
        self._traj_handler.V_inf = 0.5
        x_drone = self._drone_state[0]
        y_drone = self._drone_state[1]
        self._traj_handler.alpha_inf =  math.atan2(msg.point.y - y_drone, msg.point.x - x_drone)
        if self._state == self.States.FLOCKING or self._state == self.States.NAVIGATING or self._state == self.States.WAITING:
            self.needs_to_stop = False
            self._state = self.States.HUNTING
            self.get_logger().info("Now HUNTING")
        self._traj_handler.update_panels(self._drone_state[0], self._drone_state[1])


    def _no_lag(self, msg_time) -> bool:
        # Return True if the lag between when the message was sent and now is not too long
        diff_time = self.get_clock().now().nanoseconds/1000000 - msg_time
        if diff_time > self.lag_limit_milisec:
            self.get_logger().info(" Too much lag.. diff_time is at: " + str(diff_time) + " ms") #in s
            return False
        return True

    def _new_drone_state(self, msg_time, drone_state):
        #self.get_logger().info("drone_state is"+str(drone_state))
        self.last_msg_time = msg_time
        if self._state == self.States.INIT or \
                self._state == self.States.LANDING or \
                self._state == self.States.DONE:
            #self.get_logger().info("States: "+str(self._state))
            return

        if self._state == self.States.WAITING:
            if self.first_waiting:
                self.first_waiting = False
                self._launch(self._drone_state)
                return
            else:
                pass
        if self._state == self.States.LAUNCHING:
                self._launch(self._drone_state)            

        # process the FLOCKING state:
        self._drone_state = drone_state

        if not self._no_lag(msg_time):
            self.get_logger().info("Returning cause too much lag")
            return
        

    #Process all the interactions from the other agents, and the local ones
    def calculate_cmd_and_infos(self):
        
        #Individual variables
        vel_cmd = np.zeros(4)
        index_obst = 0
        r_obst = 10
        yaw = self._drone_state[3]
        infos=None

        if self._state == self.States.FLOCKING or self._state == self.States.HUNTING:
            r_w, out_arena, close_to_wall = self.distance_to_wall()
            theta_w = self.angle_to_wall()


            #Calculate the individual interaction
            if not self.LEAVE_ARENA:
                yaw_fluct_wall = self.fluct_and_wall(r_w, theta_w)
            else:
                yaw_fluct_wall = (0., 0.)
            yaw_fluct_obstacles, index_obst, r_obst = self.fluct_obst()
            if r_obst == None:
                r_obst = 20.
            if self.index==1:
                if self.i<=30:
                    self.i += 1
                    #print("[d_wall, theta_w, yaw_fluct_wall, yaw_fluct_obst: [", r_w, ",", theta_w, ",", yaw_fluct_wall, ",", yaw_fluct_obstacles, "]")

        elif self._state == self.States.NAVIGATING:
            yaw_fluct, out_corr = self.fluct_nav()

        #Calculate all the others interactions, with the other_drone_state list

        Yaw_ali_changes, Yaw_att_rep_changes, Vel_diff, Vz =[], [], [], (self._drone_state[2]-self._prev_pos_state[2])/self.dt
        Vx0 = (self._drone_state[0]-self._prev_pos_state[0])/self.dt
        Vy0 = (self._drone_state[1]-self._prev_pos_state[1])/self.dt
        Vx = np.cos(yaw)*Vx0 + np.sin(yaw)*Vy0
        self._prev_pos_state = self._drone_state
        max_ali_att_rep_index=0
        max_speed_index=0
        Acc_z_info = [Vz]
        total_int_compare = 0.
        Delta_Vx_max=0.
        Delta_Vz_max=0.
        Delta_yaw_dot_max=0.
        try:
            Acc_z_info += [self.params['navfish_y_perp']/(1+math.exp((self._drone_state[2]- self.params['navfish_zmin'])/self.params['navfish_dz0']))]
        except OverflowError:
            Acc_z_info += [1e5]
        try:
            Acc_z_info += [-self.params['navfish_y_perp']/(1+math.exp((self.params['navfish_zmax'] - self._drone_state[2])/self.params['navfish_dz0']))]
        except OverflowError:
            Acc_z_info += [1e5]
        if self.params['navfish_zmax'] - self._drone_state[2] < -0.5:
            Acc_z_info[-1] = -1.
            self.get_logger().info("TOO HIGH, GOING DOWNN !!!")
        elif self.params['navfish_zmin'] - self._drone_state[2] > 0.5:
            Acc_z_info[-1] = 1.
            if self._state == self.States.FLOCKING:
                self.get_logger().info("TOO LOW, GOING UP !!! height is: "+str(self._drone_state[2]))
        if self.params['navfish_alt'] != 'None' and self.params['navfish_alt'] != 0.0:
            try:
                Acc_z_info += [-self.params['navfish_y_perp'] * math.tanh((self._drone_state[2]-self.params['navfish_alt'])/self.params['navfish_a_z'])]
            except OverflowError:
                Acc_z_info += [1e5]
        if Vx>0.25:
            Acc_z_info += [-self.params['navfish_y_para']*math.sin(Vz/Vx)]
        Acc_z = sum(Acc_z_info)
        #if self.index == 2:
        #    print("[Vz, perp_min, perp_max, para]", Acc_z_info)
        #Acc_z += 0.25*math.sin((self.params['navfish_alt']-self._drone_state[2])/self.dt)
        
        if self.i_log==5:
            pass #self.get_logger().info("[Vz, perp_min, perp_max, para]"+str(Acc_z_info))
        Acc_z_collect = 0.0 
        yaw_collect_neighbors = 0.
        Vel_collect = 0.
        Acc_speed_max = 0.
        for index_active in self._other_state_active_i:
            if index_active < self.index:
                x=self._other_drone_state[index_active-1]
            else:
                x=self._other_drone_state[index_active-2]
            d_coupled = self.coupling_distance(x)
            viewing_angle = self.viewing_angle(x)
            yaw_alignment = 0.
            yaw_att_rep = 0.
            Delta_Vz = 0.
            try:
                Delta_Vx = self.params['navfish_yacc'] * math.cos(viewing_angle) * (self.params['navfish_dv0'] - d_coupled)/(1+d_coupled/self.params['navfish_lacc'])
            except FloatingPointError:
                Delta_Vx = 0.
                #Vel_diff -= vel_int
            if self._state == self.States.FLOCKING:
                yaw_alignment = self.alignment(x[3], d_coupled, viewing_angle, r_w, r_obst, close_to_wall)
                yaw_att_rep = self.attraction_repulsion(x[3], d_coupled, viewing_angle, r_w, r_obst, close_to_wall)
            elif self._state == self.States.NAVIGATING or self._state == self.States.HUNTING:
                yaw_alignment = self.alignment(x[3], d_coupled, viewing_angle, 10,r_obst, False)
                yaw_att_rep = self.attraction_repulsion(x[3], d_coupled, viewing_angle, 0,r_obst, False)
            if self._state != self.States.LAUNCHING and self._state != self.States.INIT:
                try:
                    Delta_Vz =  float(self.vertical_repulsion(self._drone_state[2],x[2],d_coupled))
                except FloatingPointError:
                    Delta_Vz = 0.
            try:
                total_int = math.sqrt( Delta_Vx**2 + ((yaw_alignment+yaw_att_rep)*self.params['navfish_velocity'])**2 + Delta_Vz**2 )
            except FloatingPointError:
                total_int = 0.
            if abs(total_int)>abs(total_int_compare):
                total_int_compare = total_int
                max_total_int_index = index_active
                Delta_Vz_max = Delta_Vz
                Delta_Vx_max = -Delta_Vx
                Delta_yaw_dot_max = yaw_alignment + yaw_att_rep
            else:
                pass
          
        if self._state == self.States.FLOCKING:
            #yaw_cmd =  yaw + (sum(yaw_fluct_wall) +yaw_fluct_obstacles+ yaw_collect_neighbors ) * self.dt
            try:
                yaw_cmd =  yaw + (sum(yaw_fluct_wall) +yaw_fluct_obstacles+ Delta_yaw_dot_max ) * self.dt
            except FloatingPointError:
                yaw_cmd = 0.
            yawdot_cmd =self._controller.yaw_control(yaw_cmd, yaw)
            vel_cmd[3] = yawdot_cmd

            self.velocity = Vx + Delta_Vx_max*self.dt
            vel_cmd[0] = np.clip(self.velocity, self.params['navfish_minvelocity'], self.params['navfish_maxvelocity'])
            #self.get_logger().info("vel cmd is: "+str(vel_cmd))

            vel_cmd[2] = (Acc_z+Delta_Vz_max)#*self.dt
            yaws=[yaw, yaw_fluct_obstacles, yaw_fluct_wall[0], yaw_fluct_wall[1], float(sum(Yaw_ali_changes)), float(sum(Yaw_att_rep_changes))]
            infos = [Acc_z_info, Delta_Vz_max, max_ali_att_rep_index, max_speed_index, index_obst, yaws, float(self._controller.PID.SetPoint), float(yawdot_cmd), 'flocking', out_arena]
            self._other_state_active_i=[]
        elif self._state == self.States.NAVIGATING: 
            yaw_traj = self._traj_handler.yaw_traj
            yaw_cmd = yaw + ( yaw_fluct + Delta_yaw_dot_max + 5.5 * math.sin(yaw_traj - yaw) ) * self.dt
            yawdot_cmd =self._controller.yaw_control(yaw_cmd, yaw)
            vel_cmd[3] = yawdot_cmd

            #Altitude control
            #hdot_cmd = self._controller.altitude_control(self.z_start, self._drone_state[2])
            vel_cmd[2] = (Acc_z+Delta_Vz_max)

            #Altitude control
            #hdot_cmd = self._controller.altitude_control(self.z_start, self._drone_state[2])
            #vel_cmd[2] = hdot_cmd
            #Configure the speed command
            self.velocity = Vx + Delta_Vx_max*self.dt
            vel_cmd[0] = np.clip(self.velocity, self.params['navfish_minvelocity'], self.params['navfish_maxvelocity'])
            # send the velocity command to the drone and the info to the info pub
            yaws=[yaw, 0., yaw_fluct, yaw_traj, float(sum(Yaw_ali_changes)), float(sum(Yaw_att_rep_changes))]
            
            #self._call_cmd(vel_cmd)
            infos = [Acc_z_info, Delta_Vz_max, max_ali_att_rep_index, max_speed_index, index_obst, yaws, float(self._controller.PID.SetPoint), float(yawdot_cmd), 'schooling', out_corr]
            #self._call_info(Acc_z_info, Acc_z_collect, max_ali_att_rep_index, index_obst, yaws, float(self._controller.PID.SetPoint), float(yawdot_cmd), 'schooling', out_corr)

        elif self._state == self.States.HUNTING:
            #self.get_logger().info("NAVIGATING or HUNTING")
            #Final command, sum of all the others (input selection above)
            yaw_traj = self._traj_handler.yaw_traj_panels
            if self._traj_handler.pitch_traj_tube!=None:
                if not self._traj_handler.doing_panels:
                    yaw_traj = yaw
                pitch_traj_tube = self._traj_handler.pitch_traj_tube
                yaw_traj_tube = -self._traj_handler.yaw_traj_tube
                #print("Yaw_traj is", yaw_traj)

                yaw_cmd =  yaw + (sum(yaw_fluct_wall) + yaw_fluct_obstacles + Delta_yaw_dot_max +yaw_traj_tube+ 7.5 * math.sin(yaw_traj - yaw)) * self.dt
                yawdot_cmd =self._controller.yaw_control(yaw_cmd, yaw)
                vel_cmd[3] = yawdot_cmd 

                #Altitude control
                #hdot_cmd = self._controller.altitude_control(self.z_start, self._drone_state[2])
                vel_cmd[2] = Acc_z+Delta_Vz_max + Vx*math.tan(pitch_traj_tube)
            else:
                yaw_cmd =  yaw + (sum(yaw_fluct_wall) + yaw_fluct_obstacles + Delta_yaw_dot_max+ 7.5 * math.sin(yaw_traj - yaw)) * self.dt
                yawdot_cmd =self._controller.yaw_control(yaw_cmd, yaw)
                vel_cmd[3] = yawdot_cmd 

                #Altitude control
                #hdot_cmd = self._controller.altitude_control(self.z_start, self._drone_state[2])
                vel_cmd[2] = Acc_z+Delta_Vz_max
            
            #print("Acc_z_info, Delta_Vz_max, Vx*math.tan(pitch_traj_tube)", Acc_z_info, Delta_Vz_max, Vx*math.tan(pitch_traj_tube))

            #Configure the speed command
            self.velocity = Vx + Delta_Vx_max*self.dt
            vel_cmd[0] = np.clip(self.velocity, self.params['navfish_minvelocity'], self.params['navfish_maxvelocity'])
            yaws=[yaw, yaw_fluct_obstacles, yaw_fluct_wall[0], yaw_fluct_wall[1], float(sum(Yaw_ali_changes)), float(sum(Yaw_att_rep_changes)), 0.0]
            
            #print("yaws", yaws)
            #self._call_cmd(vel_cmd)
            infos = [Acc_z_info, Delta_Vz_max, max_ali_att_rep_index, max_speed_index, index_obst, yaws, float(self._controller.PID.SetPoint), float(yawdot_cmd), 'hunting']
            #self._call_info(Acc_z_info, Acc_z_collect, max_ali_att_rep_index, index_obst, yaws, float(self._controller.PID.SetPoint), float(yawdot_cmd), 'hunting')

        elif self._state == self.States.HOLDING:
            self.velocity_cmd = (self._drone_state[0] -self._prev_pos_state[0])/self.dt -0.05
            vel_cmd[0] = self.velocity_cmd
            self.get_logger().info("Holding in place...")

            vel_cmd[2] = (Acc_z+Delta_Vz_max) - 0.05
            if self.velocity_cmd < 0.15:
                vel_cmd = [0.,0.,0.,0.]
                if not self.has_flipped:
                    #self._call_land()
                    self.call_flip()
                    self.get_logger().info("Waiting for the flip...")
                else:
                    self.get_logger().info("Time to land...")
                    self._call_land()
                    self._state = self.States.LANDING


            
            # send the velocity command to the drone and the info to the info pub
            #self._call_cmd(vel_cmd)
        
        elif self._state == self.States.RETURNING_HOME:
            self.i_log+=1
            home_point = [-3+self.index, 3.5]
            yaw_traj = math.atan2(home_point[1]-self._drone_state[1], home_point[0]-self._drone_state[0])
            yaw_cmd =  yaw +  2.25 * math.sin(yaw_traj - yaw) * self.dt
            yawdot_cmd =self._controller.yaw_control(yaw_cmd, yaw)
            vel_cmd[3] = yawdot_cmd 

            hdot_cmd = self._controller.altitude_control(2+self.index*3/4, self._drone_state[2])
            vel_cmd[2] = hdot_cmd
            
            infos = None
            dist_home_point = self.distance_drone_to_drone(home_point)
            vel_cmd[0] = 0.2 if dist_home_point<1.0 else 0.4
            if dist_home_point < 0.25:
                self._state = self.States.HOLDING
                self.get_logger().info("Succesfully returned to home point")
            else:
                if self.index==1 and self.i_log % 10 == 0:
                    self.get_logger().info("Returning towards home point, dist is: "+str(dist_home_point))

       

        elif self._state == self.States.WAITING or self._state == self.States.INIT:
            #Final command, sum of all the others (input selection above)
            
            #hdot_cmd =self._controller.altitude_control(self.z_start, self._drone_state[2])
            #vel_cmd[2] = hdot_cmd 
            #self._call_cmd(vel_cmd)
            pass
        return vel_cmd, infos, max_ali_att_rep_index, max_speed_index

    #Process all the interactions from the other agents, and the local ones
    def _process_interactions(self):
        vel_cmd, infos, id_voisin_ali_att, id_voisin_speed= self.calculate_cmd_and_infos()
        if self._state != self.States.FLIPPING:
            self._call_cmd(vel_cmd)
        else:
            self._call_cmd([0.,0.,0.,0.])
        self.i_log+=1
        diff_time = time.time()-self.time0_log
        if self.i_log==25:
            pass #self.get_logger().info("Time since last process_interaction is: "+str(diff_time)+"s,  so freq is around (Hz): "+str(1/diff_time))
        self.time0_log=time.time()
        if self.used_pose:
            pass#self.get_logger().info("ALREADY USED THIS POSITION!!! NOT NORMAL")
        else:
            self.used_pose=True
        #self.get_logger().info('INFO: '+str(infos))
        if infos != None:
            self._call_info(*infos)

        
    def distance_to_traj(self):
        x_home = 0.0
        y_home = 0.0
        dist = self.params['navfish_walldistance'] - np.sqrt(((self._drone_state[0] - x_home)**2) + ((self._drone_state[1] - y_home) **2 ))
        out=False
        if dist < 0:
            out=True
        return dist, out

    def distance_to_wall(self):
        x_home = self.arena_pose[0]
        y_home = self.arena_pose[1]
        dist = self.params['navfish_walldistance'] - np.sqrt(((self._drone_state[0] - x_home)**2) + ((self._drone_state[1] - y_home) **2 ))
        out = False
        close_to_wall = False
        if dist < 0:
            out=True
        if dist < self.params['navfish_lw']:
            close_to_wall = True
        return dist, out, close_to_wall

    def angle_to_wall(self):
        yaw = self._drone_state[3]
        theta = math.atan2(self._drone_state[1] - self.arena_pose[1], self._drone_state[0] - self.arena_pose[0])
        delta = yaw - theta
        if delta > math.pi:
            delta -= 2*math.pi
        if delta < -math.pi:
            delta += 2*math.pi
        if abs(delta)<0.25:
            delta = np.sign(delta)*0.25
        #self.get_logger().info("The angle to the normal of the wall is of :" +str(delta))
        return delta

    def fluct_and_wall(self, dist_wall, angle_wall):
        lw = self.params['navfish_lw']
        if lw < 0.1:
            lw = 2.45
        fw = math.exp(-(dist_wall/lw)**2)
        ow = self.params['navfish_ew1'] * math.cos(angle_wall) + self.params['navfish_ew2'] * math.cos(2*angle_wall)
        order = -angle_wall-math.pi
        if order < -math.pi:
            order += 2*math.pi
        elif order > math.pi:
            order -= 2*math.pi
        if dist_wall < -0.5:
            #self.get_logger().info('Got out too far, going straight to center, at yaw: '+str(order))
            wall = order
            fluct=0.0
        else:
            wall = self.params['navfish_yw'] * math.sin(angle_wall) * (1 + ow) * fw
            fluct = 0.0
        if self.i_log==30 or self.i_log==15:
            self.get_logger().info("dist_wall, angle_wall and wall are " +str([dist_wall, angle_wall, wall]))
            self.get_logger().info("If it were outside, would go towards yaw: " +str(order))
            self.i_log=0
        return (wall, fluct)
    
    def fluct_nav(self):
        if self._traj_handler.track_error != None and self._traj_handler.d_corr != None:
            dist_to_corr = self._traj_handler.d_corr-self._traj_handler.track_error
        else:
            dist_to_corr = 0.0
        lw = self.params['navfish_lw']
        if lw < 0.1:
            lw = 2.45
        fw = math.exp(-(dist_to_corr/lw)**2)
        out = False
        if dist_to_corr<0:
            fw=0
            out = True
            self.get_logger().info('Out of the corridor')
        fluct = self.params['navfish_fluct'] * (1 - self.params['navfish_alpha'] * fw) * self.normal_random_gen()
        
        return fluct, out

    def fluct_obst(self):
        self._traj_handler.buildings.sort(key=lambda building:math.sqrt((building.center[0]-self._drone_state[0])**2+(building.center[1]-self._drone_state[1])**2))
        lob = self.params['navfish_lw']
        yaw_obst_list = []
        index_max_obst = 0
        yaw_obst_compare = 0.
        x = self._drone_state[0]
        y = self._drone_state[1]
        yaw = self._drone_state[3]
        rw_min = None
        for building in self._traj_handler.buildings[:3]:
            if self._drone_state[2] < 0.2 + max([vertice[2] for vertice in building.vertices]): # if not above the building, not interacting
                #if self.index == 1:
                    #self.get_logger().info("The three closest buildings are at a distance of")
                rw = math.sqrt((building.center[0]-x)**2+(building.center[1]-y)**2) -building.radius
                if rw_min == None or abs(rw) < rw_min: 
                    rw_min = rw
                #if self.index == 1:
                #    self.get_logger().info(str(rw))
                fw = math.exp(-(rw/lob)**2)
                angle_to_obst = yaw- math.atan2(building.center[1]-y,building.center[0]-x)
                if angle_to_obst>2*np.pi:
                    angle_to_obst-= 2*np.pi
                if angle_to_obst<-2*np.pi:
                    angle_to_obst+= 2*np.pi
                ow = self.params['navfish_ew1_ob'] * math.cos(angle_to_obst) + self.params['navfish_ew2_ob'] * math.cos(2*angle_to_obst)
                yaw_obst = self.params['navfish_yob'] * math.sin(angle_to_obst) * (1 + ow) * fw
                if abs(yaw_obst) > abs(yaw_obst_compare):
                    yaw_obst_compare = yaw_obst
                    index_max_obst = building.index_b
                    #self.get_logger().info("Registered a building close to the agent, whose index is "+str(building.index_b))
                yaw_obst_list.append(yaw_obst)
        return float(sum(yaw_obst_list)), index_max_obst, rw_min

    #Gaussian random number generator with mean =0 and invariance =1 using Box-Muller method
    def normal_random_gen(self):
        U1 = np.random.uniform(size = 1)
        U2 = np.random.uniform(size = 1)
        R = np.sqrt(-2 * np.log(U1))
        Theta = 2 * np.pi * U2
        X = R * math.cos(Theta)
        return float(X)

    def distance_drone_to_drone(self, state_other):
        dist = np.sqrt(
            (state_other[0] - self._drone_state[0])**2 +
            (state_other[1] - self._drone_state[1])**2
        )
        return dist
    
     
    def distance3d_drone_to_drone(self, state_other):
        dist = np.sqrt(
            (state_other[0] - self._drone_state[0])**2 +
            (state_other[1] - self._drone_state[1])**2 +
            (state_other[2] - self._drone_state[2])**2
        )
        return dist
    
    def coupling_distance(self, state_other):
        dist = np.sqrt(
            (state_other[0] - self._drone_state[0])**2 +
            (state_other[1] - self._drone_state[1])**2 +
            self.params['navfish_alpha_z'] * (state_other[2] - self._drone_state[2])**2
        )
        return dist

    def viewing_angle(self, state_other): #Only works in an horizontal plane
        direction = 0.0

        diff = (
            (state_other[0] - self._drone_state[0]), 
            (state_other[1] - self._drone_state[1])
        )
        direction = math.atan2(diff[1], diff[0])
        viewing_angle = direction - self._drone_state[3]
        if viewing_angle > math.pi:
            viewing_angle -= 2*math.pi
        elif viewing_angle < - math.pi:
            viewing_angle += 2*math.pi
        return viewing_angle

    def vertical_repulsion(self,z, z_other, d_coupled):
        if self.i < 100:
            self.i+=1
        else:
            self.i=0
            #self.get_logger().info("dz is "+str(z_other-z)+", d is "+str(d2d)+", tanh(dz/az) is: "+str(math.tanh((z_other-z)/self.params['navfish_a_z']))+" and the exp is equal to: "+str(math.exp(-(d2d**2)*(z_other-z)**2)))
        #print('self.params[navfish_a_z]', self.params['navfish_a_z'])
        #sprint('diff z', z_other-z)
        if z_other - z !=0:
            return self.params['navfish_y_z'] * math.tanh((z-z_other)/self.params['navfish_a_z'])*math.exp(-(d_coupled**2)/self.params['navfish_L_z_2']) ## param 0.25
        else:
            print('exact same height.. weird')
            return 1e-2

    def alignment(self, yaw_other, dist_to_neighbor, viewing_angle, r_w = 10, r_obst=10, close_to_wall = False):
        
        psi = yaw_other - self._drone_state[3]
        if psi > math.pi:
            psi-=2*math.pi
        if psi < -math.pi:
            psi+=2*math.pi
        if not close_to_wall:
            try:
                lw = self.params['navfish_lw']
                fw = math.exp(-(r_w/lw)**2)
                exp_term = math.exp(-(dist_to_neighbor/self.params['navfish_lali'])**2) * math.sin(psi)
                yaw_alignment = self.params['navfish_yali'] * (dist_to_neighbor + self.params['navfish_d0ali'])/self.params['navfish_d0ali'] * exp_term *(1 - fw)
            except OverflowError:
                yaw_alignment=5e3
            except FloatingPointError:
                yaw_alignment = 5e-6
        else:
            lw = self.params['navfish_lw']
            if lw < 0.1:
                lw = 2.45
            lob = self.params['navfish_lob']
            fw = math.exp(-(r_w/lw)**2)
            fob = math.exp(-(r_obst/lob)**2)
            try:
                exp_term = math.exp(-(dist_to_neighbor/self.params['navfish_lali'])**2)
            except OverflowError:
                exp_term=5e3
            except FloatingPointError:
                exp_term = 5e-6
            try:
                F = self.params['navfish_yali'] * (dist_to_neighbor + self.params['navfish_d0ali']) * exp_term *(1 - self.params['navfish_alpha'] * fw) *(1 - self.params['navfish_alpha'] * fob) 
                self.params['navfish_yatt'] * (dist_to_neighbor/self.params['navfish_d0att']-1) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * exp_term
                E = 0.9012 * (1 + 0.6 *math.cos(viewing_angle) - 0.32 *math.cos(2*viewing_angle))
                O = 1.6385 * math.sin(psi) * (1 + 0.3 * math.cos(2*psi))
                yaw_alignment = F*O*E
                #print("did FOE style in ali")
            except FloatingPointError:
                return 0. 
        if self.i_log==10:
            self.get_logger().info("ALIGNMENT: psi, dist_to_neighbor, viewing_angle, r_w and yaw_alignment are: "+str([psi, dist_to_neighbor, viewing_angle, r_w, yaw_alignment]))
        return yaw_alignment
    
    def attraction_repulsion(self, yaw_other, dist_to_neighbor, viewing_angle, r_w = 10, r_obst=10, close_to_wall = False):
        if dist_to_neighbor<0:# > self.params['navfish_d0att']:
            self.get_logger().info("SHOULD NOT BE HEEEEERRREEE!!!")
            if not close_to_wall:
                yaw_att_rep = self.params['navfish_yatt'] * (dist_to_neighbor - self.params['navfish_d0att']) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * math.sin(viewing_angle) * ((self.params['navfish_latt'] - self.params['navfish_d0att'])/(dist_to_neighbor - self.params['navfish_d0att'])) *(1 - fw)
            else:
                lw = self.params['navfish_lw']
                lob = self.params['navfish_lob']
                try:
                    fw = math.exp(-(r_w/lw)**2)
                except OverflowError:
                    fw=10e5
                fob = math.exp(-(r_obst/lob)**2)
                yaw_att_rep = self.params['navfish_yatt'] * (dist_to_neighbor - self.params['navfish_d0att']) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * math.sin(viewing_angle) * ((self.params['navfish_latt'] - self.params['navfish_d0att'])/(dist_to_neighbor - self.params['navfish_d0att'])) *(1 - self.params['navfish_alpha'] * fw) *(1 - self.params['navfish_alpha'] * fob) 
                #self.get_logger().info("No error this time")
        else:
            if True:#not close_to_wall:
                #self.get_logger().info("Not close to wall, normal interaction")
                try:
                    exp_term = math.exp(self.params['navfish_d0att']/(2*dist_to_neighbor))
                    #exp_term = math.exp(- dist_to_neighbor/ (2*(dist_to_neighbor-self.params['navfish_d0att']/2) ))
                except OverflowError:
                    exp_term = 5000
                except FloatingPointError:
                    exp_term = 5e-4
                try:
                    psi = yaw_other - self._drone_state[3]
                    F = self.params['navfish_yatt'] * (dist_to_neighbor/self.params['navfish_d0att']-1) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * exp_term
                    O = 1.395 * math.sin(viewing_angle)*(1- 0.33*math.cos(viewing_angle))
                    E = 0.9326 * (1 - 0.48 * math.cos(psi) - 0.31 * math.cos(2*psi))
                    yaw_att_rep = F*O*E
                    #print("did FOE style in att")
                    #yaw_att_rep = self.params['navfish_yatt'] * (dist_to_neighbor/self.params['navfish_d0att']-1) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * math.sin(viewing_angle) * exp_term
                except FloatingPointError:
                    self.get_logger().info("Floating POINT ERROR in att_rep, here is the value of exp_term: " +str(exp_term))
                    yaw_att_rep = 1e-4
            else:
                lw = self.params['navfish_lw']
                fw = math.exp(self.params['navfish_yali']*(self.params['navfish_d0att'] - dist_to_neighbor)/self.params['navfish_d0att'])
                try:
                    exp_term = math.exp(- dist_to_neighbor/ (2*(dist_to_neighbor-self.params['navfish_d0att']/2) ))
                except OverflowError:
                    exp_term = 5000
                except FloatingPointError:
                    exp_term = 5e-3
                try:
                    yaw_att_rep = self.params['navfish_yatt'] * (dist_to_neighbor - self.params['navfish_d0att']) / (1 + (dist_to_neighbor/self.params['navfish_latt'])**2) * math.sin(viewing_angle) * exp_term *(1 - self.params['navfish_alpha'] * fw)
                except FloatingPointError:
                    self.get_logger().info("Floating POINT ERROR in att_rep, here is the value of exp_term: " +str(exp_term))
                    yaw_att_rep = 1e-4
        #if self.index == 1:
            #self.get_logger().info("Unclipped att interaction returns "+str(yaw_att_rep))
        if self.i_log==20:
            pass #self.get_logger().info("ATTRACTION: psi, dist_to_neighbor, viewing_angle, r_w and yaw_att_rep are: "+str([psi, dist_to_neighbor, viewing_angle, r_w, yaw_att_rep]))
        return np.clip(yaw_att_rep, -math.pi, math.pi)


    def _launch(self, drone_state):
        #self.get_logger().info('launching')
        self._drone_state = drone_state
        if drone_state[2]>2.5:
            self._state = self.States.FLOCKING 
        else:
            self._state = self.States.LAUNCHING
            if self.i_log %10==0:
                self.get_logger().info("Still too low, distance is about "+str(2.5-drone_state[2])+ "m")
        if self.first_launch_msg:
            self.get_logger().info("inside launch, state is "+ str(self._state))
            self.first_launch_msg = False
        if not self._already_flying:
            print("calling takeoff callback")
            self._call_takeoff()

    def _land(self):
        self.get_logger().info('landing')
        if not self._already_flying:
            self._state = self.States.LANDING
            self._call_land()


    def _call_info(self, acc_z_info, acc_z_collect, ali_att_rep_index, speed_index, index_obst, yaws, yaw_cmd, yawdot_cmd, phase, out_of_zone=False, too_close_to_wall=False, too_close_to_neighbor=False):
        info = Info()
        info.id_uav = self.index
        info.id_interactive_neighbor_ali_att_rep = ali_att_rep_index
        info._id_interactive_obstacle = index_obst
        info.acc_z_info = acc_z_info
        info.acc_z_collect = acc_z_collect
        info.yawdot_cmd = yawdot_cmd
        info.yaw_cmd = yaw_cmd
        info.phase = phase
        info.yaw = yaws[0]
        info.yaw_obstacle = yaws[1]
        if phase == 'flocking':
            info.yaw_wall = yaws[2]
        info.yaw_fluct = yaws[3]
        info.yaw_ali = yaws[4]
        info.yaw_att_rep = yaws[5]
        info.out_of_zone = out_of_zone
        info.too_close_to_wall = too_close_to_wall
        info.too_close_to_neighbor = too_close_to_neighbor
        info.phase = phase
        try:
            self._info_pub.publish(info)
        except rclpy.handle.InvalidHandle:
            print('didnt pub')

    def _call_cmd(self, vel_cmd):
        twist = Twist()
        twist.linear.x = vel_cmd[0]
        twist.linear.y = vel_cmd[1]
        twist.linear.z = vel_cmd[2]
        twist.angular.z = vel_cmd[3]
        self.last_sent_time_ns = self.get_clock().now().nanoseconds
        if self._state != self.States.INIT and self._state != self.States.WAITING and self._state != self.States.LAUNCHING:
            try:
                self._cmd_vel_pub.publish(twist)
            except rclpy.handle.InvalidHandle:
                print('didnt pub')
        elif self._state == self.States.LAUNCHING:
            twist.linear.x = 0.
            twist.linear.y = 0.
            twist.linear.z = 0.325
            twist.angular.z = 0.
            #self.get_logger().info("Agent "+str(self.index)+" should now be going up in launch...")
            try:
                self._cmd_vel_pub.publish(twist)
                if self.i_log==15:
                    self.get_logger().info("Going to the right altitude before starting. . .")
            except rclpy.handle.InvalidHandle:
                print('didnt pub')




    def _call_takeoff(self):
        action = Action()
        #self.get_logger().info('taking off')
        action.index = self.index
        action.action = 'takeoff'
        self._already_flying = True
        try:
            self._action_pub.publish(action)
        except rclpy.handle.InvalidHandle:
            print('didnt pub')
        msg = Order()
        msg.index = self.index
        msg.order= 'takeoff'
        self.order_pub_.publish(msg)

    def _flight_data_callback(self, msg):
        if msg.bat < 15 :
            self.get_logger().info("LOW BATTT!!!!")
            self._land()

    def _call_land(self):
        self._stae = self.States.LANDING
        action = Action()
        action.index = self.index
        action.action = 'land'
        self._action_pub.publish(action)
        msg = Order()
        msg.index = self.index
        msg.order= 'land'
        self.order_pub_.publish(msg)

    def call_flip(self):
        if self.flip_call_count < 25:
            self.flip_call_count+=1
            self._state = self.States.FLIPPING
            msg = Order()
            msg.index = self.index
            msg.order= 'flip b'
            self.get_logger().info("Sending flip")
            self.order_pub_.publish(msg)
        else:
            self._call_land()

    def _ros_start_callback(self, msg):
        if self._state == self.States.INIT:
            self.start()

    def _ros_stop_callback(self, msg):
        self.stop()


#------------Main()---------------------------------------

def main(args=None):
    rclpy.init(args=args)

    id_uav = int(sys.argv[1])
    nb_uav = int(sys.argv[2])
    z_start = float(sys.argv[3])

    agent = Agent(id_uav, nb_uav, z_start)

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        agent.get_logger().info("Ctrl-C detected, shutting down")
        agent.stop()
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
