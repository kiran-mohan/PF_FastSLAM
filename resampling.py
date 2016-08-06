# This code can be worked on for the resampling part of the algorithm
# Assuming the input is the particles i.e [p1,p2,p3]

import random

def ssum(k,particles):
    sum = 0
    for i in range(k+1):
        sum = sum + particle_weights[i]
    return sum

def sus(particles):
    f = 360
    n = len(particles)
    p = f/n
    start = random.randint(0,p)
    pointers = []
    for i in range(n):
        pointers.append(start + i*p)
    return rws(pointers,particles)

def rws(pointers,particles):
    keep = []
    for pointer in pointers:
        k = 0
        while (ssum(k,particles)) < pointer:
            k += 1
        keep.append(particles[k][4])
    return keep

#INPUT
p1 = [0,0,0,1,1]
p2 = [0,0,0,0,2]
p3 = [0,0,0,0,3]
particles = [p1,p2,p3]

#Extracting Weights
particle_weights = []
for i in range(len(particles)):
    particle_weights.append(particles[i][3]*360)

#Resampling
print sus(particles)
