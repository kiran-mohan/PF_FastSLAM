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
        for n in range(1,int(range_max/grid_size)+1):
        #for n in range(1,int(laser.range_max/grid_size)):
            x_pos = int(np.round(x + n*grid_size*np.cos(i)))
            y_pos = int(np.round(y + n*grid_size*np.sin(i)))
            if particle_map[x_pos][y_pos] > 0.75:
		distances.append(np.linalg.norm([x_pos-x,y_pos-y]))
                break
            elif n == int(range_max/grid_size):
            #elif n == int(laser.range_max/grid_size):
                distances.append('OOR')
    #return distances
    print "{} \n".format(distances)
particle_map = np.array([[0,0,0,1,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]])
#particle_map = np.ones((5,5))
print "\nGiven grid is : \n {} \n".format(particle_map)
pos = [1,1,np.pi/2]
range_max = 2
print "Distances for -45 to +45 degrees beam with resolution of 15 degrees is:"
raytrace(pos,particle_map)
