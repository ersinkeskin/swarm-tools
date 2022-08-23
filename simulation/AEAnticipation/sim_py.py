#!/usr/bin/env python
# coding: utf-8

# imports for simulation
import math
import numpy

# posn & ort
iterLim = 130000  # iteration count

ts = 0.1  # step time
vel = 0.002 # velocity of agents
alpha = 0.004 # coefficients that relate force to linear velocities
beta = 0.12 # coefficients that relate force to angular velocities
K = -5.0 # spring constant
b = -150*1.0 # linear damper coefficient for anticipation

sideLength = 10 # agent count on a single edge of the hexagon
nAgents = 3*sideLength*sideLength-3*sideLength+1 # total number of agents

posx = numpy.zeros((nAgents,1)) # prelocate for agents position on x
posy = numpy.zeros((nAgents,1)) # prelocate for agents position on y
ort = numpy.zeros((nAgents,1))

count = 0
for i in range(1, 2*sideLength): # row
    for j in range(1, (2*sideLength-1)-abs(i-sideLength) +1): # col
        posx[count] = j - ((2.0*sideLength-1.0)-abs(i-sideLength) % sideLength)*0.50
        posy[count] = float(i)*math.sqrt(3.0)/2.0
        count = count+1

posx = posx - numpy.mean(posx)
posy = posy - numpy.mean(posy)

pos2link = numpy.zeros((nAgents*6,nAgents))
count = 0
for i in range(0, nAgents-1): # row
    for j in range(i+1, nAgents): # col
        dist = math.sqrt((posx[i]-posx[j])*(posx[i]-posx[j]) + (posy[i]-posy[j])*(posy[i]-posy[j]))
        if dist <=1.1:
            pos2link[count,i] = 1
            pos2link[count,j] = -1;
            count = count + 1;

nLinks = count

pos2link = pos2link[0:nLinks,:]
# pos2link = sparse(pos2link) # todo
link2pos = numpy.transpose(pos2link)
distLink1 = numpy.ones((nLinks,1))
velAgent = numpy.ones((nAgents,1))*vel

posx_log = numpy.zeros((iterLim, nAgents))
posy_log = numpy.zeros((iterLim, nAgents))
velx_log = numpy.zeros((iterLim, nAgents))
vely_log = numpy.zeros((iterLim, nAgents))
ort_log = numpy.zeros((iterLim, nAgents))

# 
beta_ts = beta*ts
bTs = b/ts

# obstacle
obsX = 15.00
obsY = 9.00
obsR = 3.00

obsD = 6.00
obsK = 1.40
obsList = numpy.array([[obsX, obsY, obsR],[obsX, -obsY, obsR]])

for i in range(0, iterLim):
    xLink = numpy.dot(pos2link, posx)
    yLink = numpy.dot(pos2link, posy)
    distLink = numpy.sqrt(numpy.square(xLink) + numpy.square(yLink))
    
    F = numpy.divide(bTs*(distLink-distLink1) + K*(distLink-1), distLink)
    distLink1 = distLink
    agentFx = numpy.dot(link2pos, numpy.multiply(F, xLink))
    agentFy = numpy.dot(link2pos, numpy.multiply(F, yLink))
    
    # obstacle
    for j in range(0, obsList.shape[0]):
        xObs = posx-obsList[j,0]
        yObs = posy-obsList[j,1]
        rObs = numpy.sqrt(numpy.square(xObs) + numpy.square(yObs))
        fObs = numpy.multiply(numpy.divide(obsK, numpy.square(rObs-obsList[j,2])), rObs<(obsList[j,2]+obsD))
        
        agentFx = agentFx + numpy.multiply(fObs, numpy.divide(xObs, rObs))
        agentFy = agentFy + numpy.multiply(fObs, numpy.divide(yObs, rObs))
        
    # itgt
    cos_ort = numpy.cos(ort)
    sin_ort = numpy.sin(ort)
    
    vSetp = (numpy.multiply(agentFx, cos_ort) + numpy.multiply(agentFy, sin_ort))*alpha + vel
    velAgent = velAgent + (vSetp-velAgent)*ts
    velx = numpy.multiply(velAgent, cos_ort)
    vely = numpy.multiply(velAgent, sin_ort)
    posx = posx + velx*ts
    posy = posy + vely*ts
    ort = ort + beta_ts*(-numpy.multiply(agentFx, sin_ort) + numpy.multiply(agentFy, cos_ort))
    posx_log[i] = numpy.transpose(posx)
    posy_log[i] = numpy.transpose(posy)
    velx_log[i] = numpy.transpose(velx)
    vely_log[i] = numpy.transpose(vely)
    ort_log[i] = numpy.transpose(ort)

# For visualization purpose only
import time
import pylab
from IPython import display

gama =  numpy.arange(-3.14, 3.14, 0.01)
xlimit = numpy.array([numpy.min(posx_log), numpy.max(posx_log)])*1.1
ylimit = numpy.array([numpy.min(posy_log), numpy.max(posy_log)])*1.1

for i in range(0, iterLim, 500):
    pylab.clf()
    x = posx_log[i]
    y = posy_log[i]
    vx = velx_log[i]
    vy = vely_log[i]
        
    pylab.scatter(x, y, marker = "o")
    
    for j in range(0, nAgents):
        pylab.arrow(x[j], y[j], vx[j], vy[j], width = 0.2, ec = 'red')
    
    for k in range(0, obsList.shape[0]):
        pylab.plot(obsList[k,0]+obsList[k,2]*numpy.cos(gama), obsList[k,1]+obsList[k,2]*numpy.sin(gama), 'g')
    
    pylab.xlim(xlimit)
    pylab.ylim(ylimit)
    display.clear_output(wait=True)
    display.display(pylab.gcf())
    time.sleep(0.001)




