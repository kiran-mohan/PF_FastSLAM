##### ROUGH RAY TRACING CODE #########

import numpy as np
#def raytrace(pos,particle_map,laser):
def raytrace(pos,particle_map):
    x,y,theta = pos[0],pos[1],pos[2]
#    ang_min = theta + laser.angle_min
#    ang_max = theta + laser.angle_max
#    angle_increment = laser.angle_increment

    ang_min = theta - 0.7854
    ang_max = theta + 0.7854
    angle_increment = 0.2618
    grid_size = 1
    distances = []
    for i in np.arange(ang_min,ang_max,angle_increment):
        for n in range(1,int(range_max/grid_size)):
        #for n in range(1,int(laser.range_max/grid_size)):
            x_pos = int(x + n*grid_size*np.cos(i))
            y_pos = int(y + n*grid_size*np.sin(i))
            if particle_map[x_pos][y_pos] > 0.75:
                distances.append(np.linalg.norm([x_pos-x,y_pos-y]))
                break
            elif n == int(range_max/grid_size)-1:
            #elif n == int(laser.range_max/grid_size):
                distances.append('out')
    #return distances
    print distances
particle_map = np.array([[0,0,1],[0,0,1],[0,0,1]])
pos = [1,1,np.pi/2]
range_max = 2
raytrace(pos,particle_map)
