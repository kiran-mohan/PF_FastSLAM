import numpy as np

def prob_to_logodds(prob):
    ## Assuming that prob is a scalar
    prob = np.matrix(prob,float)
    logOdds = prob/(1+prob)
    return np.asscalar(logOdds)

def logOdds_to_prob(logOdds):
    ## Assuming that logOdds is a matrix
    p = 1 - 1/(1 + np.exp(logOdds))
    return p

def swap(a,b):
    x,y = b,a
    return x,y

def bresenham2(mycoords):
    mycoords = np.array(mycoords)
    x = np.round(mycoords[:,0])
    y = np.round(mycoords[:,1])
    steep = (abs(y[1]-y[0]) > abs(x[1]-x[0]))
    if steep:
	x,y = swap(x,y)
    if x[0]>x[1]:
	x = np.array(swap(x[0],x[1]))
	y = np.array(swap(y[0],y[1]))	
    delx = x[1] - x[0]
    dely = abs(y[1] - y[0])
    error = 0
    x_n,y_n = x[0],y[0]

    X = np.zeros(delx+1)
    Y = np.zeros(delx+1)

    if y[0] < y[1]:
	ystep = 1
    else:
	ystep = -1
    for n in range(delx+1):
	if steep:
	    X[n],Y[n] = x_n,y_n
	else:
	    X[n],Y[n] = y_n,x_n
	x_n += 1
	error += dely
	if error<<1 >= delx:
	    y_n += ystep
	    error -= delx
    X,Y = swap(X,Y)
    return X,Y

def pose_world_to_map(pntsWorld,gridSize):
    pntsMap = [x/gridSize for x in pntsWorld]
    return pntsMap

def laser_world_to_map(laserEndPnts, gridSize):
    pntsMap = laserEndPnts/gridSize
    return pntsMap

def v2t(v):
    x = v[0]
    y = v[1]
    th = v[2]
    trans = np.matrix([[cos(th), -sin(th), x],[sin(th), cos(th), y],[0, 0, 1]])
    return trans

def laser_to_xy(rl):
    numBeams = len(rl.ranges)
    maxRange = rl.range_max
    # apply the max range
    idx = []
    for i in range(len(rl.ranges)):
        if isnan(rl.ranges[i]):
	    idx.append(False)
	else:
            if rl.ranges[i]>0 and rl.ranges[i]<maxRange:
	        idx.append(True)
            else:
                idx.append(False)

    if (subsample):
    	idx(2:2:end) = 0

    lins = np.linspace(rl.angle_min, rl.angle_max, numBeams)
    for i in range(len(idx)):
	if idx[i]==True:
	    angles.append(lins[i])
    
    ranges_tmp = np.array(rl.ranges)
    idx_tmp = np.array(idx)
    points = np.matrix([[ranges_tmp[idx_tmp] * np.cos(angles)], [ranges_tmp[idx_tmp] * np.sin(angles)], [np.ones(1, len(angles))]])
    
#   laser_offset to be defined as [x_diff, y_diff, th_diff] = position vector from laser to robot base
    transf = v2t(laser_offset)

    # apply the laser offset
    points = transf * points
    return points
