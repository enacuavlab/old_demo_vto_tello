import numpy as np
from datetime import datetime
from itertools import compress

from building import Building
from vehicle import Vehicle
#import pdb

class ArenaMap():
    def __init__(self,version = 0):
        self.panels = None
        self.wind = [0,0]
        self.windT = 0
        if version == 65:
            self.buildings = [Building([[3.0, 2.0, 4.2], [2.75, 1.567, 4.2], [2.25, 1.567, 4.2], [2.0, 2.0, 4.2], [2.25, 2.433, 4.2], [2.75, 2.433, 4.2]]), #AddCircularBuilding( 2.5, 2, 6, 0.5, 1.2, angle = 0)
                            Building([[1.0, 3.0, 4.5], [0.75, 2.567, 4.5], [0.25, 2.567, 4.5], [0.0, 3.0, 4.5], [0.25, 3.433, 4.5], [0.75, 3.433, 4.5]]), #AddCircularBuilding( 0.5, 3, 6, 0.5, 1.5, angle = 0)
                            Building([[1.0, 0.5, 4], [0.75, 0.067, 4], [0.25, 0.067, 4], [0.0, 0.5, 4], [0.25, 0.933, 4], [0.75, 0.933, 4]]), #AddCircularBuilding( 0.5, 0.5, 6, 0.5, 2, angle = 0)  
                            Building([[-2.65, 1.5, 4.5], [-3.0, 1.15, 4.5], [-3.35, 1.5, 4.5], [-3.0, 1.85, 4.5]]), #AddCircularBuilding( -3, 1.5, 4, 0.35, 1.5, angle = 0)
                            Building([[-2.65, -1.5, 4.5], [-3.0, -1.85, 4.5], [-3.35, -1.5, 4.5], [-3.0, -1.15, 4.5]]), #AddCircularBuilding( -3, -1.5, 4, 0.35, 1.5, angle = 0) 
                            Building([[-1.15, -0.2, 4.5], [-1.5, -0.55, 4.5], [-1.85, -0.2, 4.5], [-1.5, 0.15, 4.5]]), #AddCircularBuilding( -1.5, -0.2, 4, 0.35, 1.5, angle = 0)
                            Building([[1.5, -2.5, 4.2], [1, -2.5, 4.2], [1, -1.4, 4.2], [1.5, -1, 4.2]]),
                            Building([[3.5, -2.5, 4.2], [3, -2.5, 4.2], [3, -1, 4.2], [3.5, -1.4, 4.2]])]

    def Inflate(self, visualize = False, radius = 1e-4):
        for building in self.buildings:
            building.inflate(rad = radius)

    def Panelize(self,size):
        for building in self.buildings:
            building.panelize(size)

    def Calculate_Coef_Matrix(self,method = 'Vortex'):
        for building in self.buildings:
            building.calculate_coef_matrix(method = method)


def Flow_Velocity_Calculation(vehicles, arenamap, method = 'Vortex', update_velocities = True, output_Vsum= False):

    starttime = datetime.now()
    
    # Calculating unknown vortex strengths using panel method:
    for f,vehicle in enumerate(vehicles):
        # Remove current vehicle from vehicle list. 

        othervehicleslist = vehicles[:f] + vehicles[f+1:]
     

        # Remove buildings with heights below cruise altitue:
        vehicle.altitude_mask = np.zeros(( len(arenamap.buildings) )) #, dtype=int) 
        for index,panelledbuilding in enumerate(arenamap.buildings):
            if (panelledbuilding.vertices[:,2] > vehicle.altitude).any():
                vehicle.altitude_mask[index] = 1
        related_buildings = list(compress(arenamap.buildings,vehicle.altitude_mask))

        # Vortex strenght calculation (related to panels of each building):
        for building in related_buildings:
            building.gamma_calc(vehicle,othervehicleslist,arenamap,method = method)

    #--------------------------------------------------------------------
    # Flow velocity calculation given vortex strengths:
    flow_vels = np.zeros([len(vehicles),3])

    V_gamma   = np.zeros([len(vehicles),2]) # Velocity induced by vortices
    V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
    V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
    V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source
    V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
    V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
    V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector

    W_sink    = np.zeros([len(vehicles),1]) # Velocity induced by 3-D sink element
    W_source  = np.zeros([len(vehicles),1]) # Velocity induced by 3-D source element
    W_flow    = np.zeros([len(vehicles),1]) # Vertical velocity component (to be used in 3-D scenarios)
    W_sum     = np.zeros([len(vehicles),1])
    W_norm    = np.zeros([len(vehicles),1])
    W_normal  = np.zeros([len(vehicles),1])

    for f,vehicle in enumerate(vehicles):

        
        # Remove current vehicle from vehicle list
        othervehicleslist = vehicles[:f] + vehicles[f+1:]
        
        # Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:
        V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
        V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
        # Velocity induced by 3-D point sink. Katz&Plotkin Eqn. 3.25
        W_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[2]-vehicle.goal[2]))/(4*np.pi*(((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2+(vehicle.position[2]-vehicle.goal[2])**2)**1.5))

        # Velocity induced by 2D point source, eqn. 10.2 & 10.3 in Katz & Plotkin:
        for othervehicle in othervehicleslist:
            V_source[f,0] += (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
            V_source[f,1] += (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
            W_source[f,0] += (othervehicle.source_strength*(vehicle.position[2]-othervehicle.position[2]))/(4*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2+(vehicle.position[2]-othervehicle.position[2])**2)**(3/2))

        if method == 'Vortex':
            for building in arenamap.buildings:
                u = np.zeros((building.nop,1))
                v = np.zeros((building.nop,1))
                if vehicle.ID in building.gammas.keys():
                    # Velocity induced by vortices on each panel: 
                    global a, b, c, d, e
                    
                    u = ( building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[1]-building.pcp[:,1]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2)) ####
                    v = (-building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[0]-building.pcp[:,0]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2))
                    V_gamma[f,0] = V_gamma[f,0] + np.sum(u) 
                    V_gamma[f,1] = V_gamma[f,1] + np.sum(v)
                    """
                    for m in range(building.nop):  
                        u = ( building.gammas[vehicle.ID][m]/(2*np.pi))  *((vehicle.position[1]-building.pcp[m,1]) /((vehicle.position[0]-building.pcp[m,0])**2+(vehicle.position[1]-building.pcp[m,1])**2)) ####
                        v = (-building.gammas[vehicle.ID][m]/(2*np.pi))  *((vehicle.position[0]-building.pcp[m,0]) /((vehicle.position[0]-building.pcp[m,0])**2+(vehicle.position[1]-building.pcp[m,1])**2))
                        V_gamma[f,0] = V_gamma[f,0] + u
                        V_gamma[f,1] = V_gamma[f,1] + v
                    """
                    a = building.gammas[vehicle.ID]
                    
                    b = building.pcp   
                    c = u
                    d = v 
                    e = V_gamma
        elif method == 'Source':
            for building in arenamap.buildings:
                if vehicle.ID in building.gammas.keys():
                    # Velocity induced by vortices on each panel:       
                    XYZ2 = building.panels                      # Coordinates of end point of panel 
                    XYZ1 = np.roll(building.panels,1,axis=0)    # Coordinates of the next end point of panel                                               
                    for m in range(building.nop ):
                        # Convert collocation point to local panel coordinates:
                        xt  = vehicle.position[0] - XYZ1[m][0]
                        yt  = vehicle.position[1] - XYZ1[m][1]
                        x2t = XYZ2[m][0] - XYZ1[m][0]
                        y2t = XYZ2[m][1] - XYZ1[m][1]
                        x   =  xt * np.cos(building.pb[m]) + yt  * np.sin(building.pb[m])
                        y   = -xt * np.sin(building.pb[m]) + yt  * np.cos(building.pb[m])
                        x2  = x2t * np.cos(building.pb[m]) + y2t * np.sin(building.pb[m])
                        y2  = 0
                        # Find R1,R2,TH1,TH2:
                        R1  = (     x**2 +      y**2)**0.5
                        R2  = ((x-x2)**2 + (y-y2)**2)**0.5
                        T1 = np.arctan2( ( y    ) , ( x    ) )
                        T2 = np.arctan2( ( y-y2 ) , ( x-x2 ) )

                        up = ( building.gammas[vehicle.ID][m]/(2*np.pi)) * np.log(R1/R2)
                        vp = ( building.gammas[vehicle.ID][m]/(2*np.pi)) * (T2-T1)
                        V_gamma[f,0] = V_gamma[f,0] + up * np.cos(building.pb[m]) + vp * np.sin(building.pb[m])
                        V_gamma[f,1] = V_gamma[f,1] - up * np.sin(building.pb[m]) + vp * np.cos(building.pb[m])      
        elif method == 'Hybrid':
            pass  

        # Total velocity induced by all elements on map:
        V_sum[f,0] = V_gamma[f,0] + V_sink[f,0] + vehicle.V_inf[0] + V_source[f,0]
        V_sum[f,1] = V_gamma[f,1] + V_sink[f,1] + vehicle.V_inf[1] + V_source[f,1]

        # L2 norm of flow velocity:
        V_norm[f] = (V_sum[f,0]**2 + V_sum[f,1]**2)**0.5
        # Normalized flow velocity:
        V_normal[f,0] = V_sum[f,0]/V_norm[f]
        V_normal[f,1] = V_sum[f,1]/V_norm[f]

        # Flow velocity inversely proportional to velocity magnitude:
        V_flow[f,0] = V_normal[f,0]/V_norm[f] 
        V_flow[f,1] = V_normal[f,1]/V_norm[f]

        W_sum[f] = W_sink[f] + W_source[f]
        if W_sum[f] != 0.:
                W_norm[f] = (W_sum[f]**2)**0.5
                W_normal[f] = W_sum[f] /W_norm[f]
                W_flow[f] = W_normal[f]/W_norm[f]
                W_flow[f] = np.clip(W_flow[f],-0.07, 0.07)
        else: 
                W_flow[f] = W_sum[f]

        flow_vels[f,:] = [V_flow[f,0] + arenamap.wind[0]/(1.35*1.35), V_flow[f,1] + arenamap.wind[1]/(1.35*1.35), W_flow[f,0]]

    if output_Vsum:
        return flow_vels, V_sum
    else:
        return flow_vels
