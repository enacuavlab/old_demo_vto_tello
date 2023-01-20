These are minimal code implementing test conditions with or without several vehicles and buildings

Questions:

1) Why BuildingOut/gamma_calc implements code 

"
    for i,othervehicle in enumerate(othervehicles) :
      vel_source[:,0] += (othervehicle.source_strength*(self.pcp[:,0]-othervehicle.position[0]))/ \
        (2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))

      vel_source[:,1] += (othervehicle.source_strength*(self.pcp[:,1]-othervehicle.position[1]))/ \
        (2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))
"

as it is already implemented in Flow_Velocity_Calculation

"
    for othervehicle in othervehicleslist:
      V_source[f,0] += (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/ \
        (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))

      V_source[f,1] += (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/ \
        (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
"  
