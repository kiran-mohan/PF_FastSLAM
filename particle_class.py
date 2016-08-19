from numpy.random import normal
import numpy as np
from math import sin,cos
from helper_func import *
from resampling import resampling

class particle:
    pose = np.array([0,0,0])
    grid_map = np.matrix(np.ones((200,200)))
    weight = 0.0
    probOcc = 0.9
    probFree = 0.35
    gridSize = 0.05
    logOddsPrior = prob_to_logOdds(prior)

    def __init__(self,num_particles):
	self.weight = 1.0/num_particles	
	prior = 0.5
        self.grid_map = self.logOddsPrior*self.grid_map

    def sample_motion_model(self,ut):
        # Inputs : ut = [v w]'
        #          x_pre = [x y theta]'
        # v = 1
        # w = 0
        [v,w] = ut
        [x,y,theta] = self.pose
        a1 = 0.01
        a2 = 0.01
        a3 = 0.01
        a4 = 0.01
        a5 = 0.01
        a6 = 0.01
        del_t = 1
        v_cap = v + sample(a1*v**2 + a2*w**2)
        w_cap = w + sample(a3*v**2 + a4*w**2)
        gamma_cap = sample(a5*v**2 + a6*w**2)
        x_prime = x - (v_cap/w_cap)*sin(theta) + v_cap/w_cap*sin(theta + w_cap*del_t)
        y_prime = y + (v_cap/w_cap)*cos(theta) - v_cap/w_cap*cos(theta + w_cap*del_t)
        theta_prime = theta + w_cap*del_t + gamma_cap*del_t
        xt = [x_prime, y_prime, theta_prime]
        return xt

    def inv_sensor_model(self,scan):
        ### "scan" has to be of type sensor_msgs/LaserScan message
        ### Assume that scan is a python list
        mapUpdate = np.zeros(np.shape(self.grid_map))
        robMapPose = pose_world_to_map(self.pose[2],self.gridSize)
        robTrans = v2t(self.pose)
        laserEndPnts = laser_to_xy(scan)
        laserEndPnts = robTrans*laserEndPnts # convert from laser – robot coords
        laserEndPntsMapFrame = laser_world_to_map(laserEndPnts[:2],gridSize)
    	
        freeCells = []
        for col in range(len(laserEndPntsMapFrame[0])):
            [X,Y] = bresenham2(robMapPose[0], robMapPose[1],laserEndPointsMapFrame[0][col],laserEndPointsMapFrame[1][col])
            freeCells.append([X,Y])

        for freeCell in freeCells:
	    mapUpdate[freeCell[0]][freeCell[1]] = prob_to_logodds(self.probFree)
        for endPnt in LaserEndPntsMapFrame.transpose():
	    mapUpdate[endPnt[0]][endPnt[1]] = prob_to_logodds(self.probOcc)
	return mapUpdate, robMapPose, laserEndPntsMapFrame

    def observation_model(self,scan):
        zt = scan.ranges
        d = ray_trace(scan,self.pose,self.grid_map)
        weight = 1.0/(np.linalg.norm(zt-d) + 1)
        return weight

    def raytrace(self,scan,pos,particle_map):
	thresh_occ = 0.75
        x,y,theta = pos[0],pos[1],pos[2]
        ang_min = theta + scan.angle_min
        ang_max = theta + scan.angle_max
        angle_increment = scan.angle_increment
        distances = []
        for i in np.arange(ang_min,ang_max,angle_increment):
            for n in range(1,int(scan.range_max/self.grid_size)):
                x_pos = int(np.round(x + n*self.grid_size*np.cos(i)))
                y_pos = int(np.round(y + n*self.grid_size*np.sin(i)))
                if particle_map[x_pos][y_pos] > thresh_occ:
    	  	    distances.append(np.linalg.norm([x_pos-x,y_pos-y]))
                    break
                elif n == int(scan.range_max/self.grid_size):
                    distances.append('OOR')
        return distances

if __name__ == '__main__':
    num_particles = 30
    particles = [particle(num_particles) for i in range(num_particles)]
    while(not ut.end):
    ### ut, scan = Subscribe to topics to get twist and laser scan after delta_t
        for i in range(num_particles):
	    particles[i].pose = particles[i].sample_motion_model(ut)
    	    particles[i].weight = observation_model(scan)
            [mapUpdate, robPoseMap, laserEndPntsMapFrame] = inverse_sensor_model(scan)
	    particles[i].grid_map -= logOddsPrior	
	    particles[i].grid_map += mapUpdate
	    plot_map(particles[i].grid_map,robPoseMap, laserEndPntsMapFrame, particles[i].gridSize)
	particles = resampling(particles)
