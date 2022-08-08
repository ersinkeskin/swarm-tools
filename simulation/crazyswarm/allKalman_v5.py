"""Takeoff-hover-land for one CF. Useful to validate hardware config."""
from numpy import ndarray

from pycrazyswarm import Crazyswarm
import numpy as np
import scipy.io as sio
import scipy.linalg as sli
from numpy.linalg import inv
import time
from matplotlib import pyplot as plt
import math
#import keyboard

TAKEOFF_DURATION = 3.5


def main():
    open("logKalman.txt","w").close() #ac, sil, kapat
    fh = open("logKalman.txt","a")
    exceptionCase = 0
    distThreshold = 0.3*1
    arenaHardLimit = 2.5

    addNoise = 0

    iterLim = 200
    nAgents = 6
    iFocal = 3

    dn = 0.8
    ts = 0.05*2
    sleepRate = 1/ts
    tsim = ts*iterLim
    vel = 0.3
    alpha = 0.1
    beta = 0.4
    Ksp = vel/(dn*0.3*alpha)
    Kacc = 0.1
    b = 10*1
    sigmaPos = 0.005
    sigmaOrt = 5 * np.pi/180

    q1 = 0.01
    q3 = 0.01
    q4 = 0.1
    q3o = q3*1
    q4o = q4*2

    ts2 = ts**2
    ts3 = ts**3

    #obstacle
    obsX = 1.6 -0.3*0
    obsR = 1.0
    obsD = obsX-obsR
    obsY = 0.0

    obsK = 0.15/3 *0;
    maxFobs = vel/alpha*1.5

    # pozisyonlar

    posx = np.zeros(shape=(6,1))
    posy = np.zeros(shape=(6,1))
    posz = np.zeros(shape=(6,1))

    posx[0] = 0.85*np.cos(np.pi / 10 + 3*2*np.pi/5)
    posx[1] = 0.85*np.cos(np.pi / 10 + 4*2*np.pi/5)
    posx[2] = 0.85*np.cos(np.pi / 10 + 2*2*np.pi/5)
    posx[3] = 0.0
    posx[4] = 0.85*np.cos(np.pi / 10 + 0*2*np.pi/5)
    posx[5] = 0.85*np.cos(np.pi / 10 + 1*2*np.pi/5)

    posy[0] = 0.85*np.sin(np.pi / 10 + 3*2*np.pi/5)
    posy[1] = 0.85*np.sin(np.pi / 10 + 4*2*np.pi/5)
    posy[2] = 0.85*np.sin(np.pi / 10 + 2*2*np.pi/5)
    posy[3] = 0.0
    posy[4] = 0.85*np.sin(np.pi / 10 + 0*2*np.pi/5)
    posy[5] = 0.85*np.sin(np.pi / 10 + 1*2*np.pi/5)

    posx = posx * dn
    posy = posy * dn

    posy = posy - 1.5

    ort = np.ones((nAgents, 1))*np.pi/2
    velAgent = np.ones((nAgents, 1))*vel

    #centerIdsArr = dict()
    centerIdsArr = [np.array([0, 1, 2, 3]), np.array([1, 0, 3, 4]), np.array([2, 0, 3, 5]),
                    np.array([3, 0, 1, 2, 4, 5]), np.array([4, 1, 3, 5]), np.array([5, 2, 3, 4])]

    nRobotsArr = np.array([4, 4, 4, 6, 4, 4])

    # KF
    n = 4

    H1 = np.zeros(shape=(3, n))
    H1[0, 0] = 1
    H1[1, 1] = 1
    H1[2, 3] = 1

    H2 = np.zeros(shape=(2, n))
    H2[0, 0] = 1
    H2[1, 1] = 1

    H = sli.block_diag(H1, H2, H2, H2, H2, H2, H2)
    for i in range(4, 15, 2):
        H[i - 1, 0] = -1*1
        H[i, 1] = -1*1

    H6 = H
    H5 = H[0:(3 + 5 * 2), 0: (n * 6)]
    H4 = H[0:(3 + 4 * 2), 0: (n * 5)]
    H3 = H[0:(3 + 3 * 2), 0: (n * 4)]

    Rk = np.identity(15) * sigmaPos ** 2
    Rk[2, 2] = sigmaOrt ** 2

    Rk6 = Rk
    Rk5 = Rk[0:np.shape(H5)[0], 0:np.shape(H5)[0]]
    Rk4 = Rk[0:np.shape(H4)[0], 0:np.shape(H4)[0]]
    Rk3 = Rk[0:np.shape(H3)[0], 0:np.shape(H3)[0]]

    #xks = dict()
    #Pks = dict()
    xk = np.zeros((nAgents,1))
    Pk = np.zeros((nAgents*n,nAgents*n))
    xks = [xk, xk, xk, xk, xk, xk]
    Pks = [Pk, Pk, Pk, Pk, Pk, Pk]

    #initialize
    ort1 = np.zeros(shape=(iterLim,1))
    ort2 = np.zeros(shape=(iterLim,1))

    for ik in range(0,nAgents):
        nRobots = nRobotsArr[ik]
        centerIds = centerIdsArr[ik]
        xk = np.zeros((nRobots*n,1))
        for ir in range(0,nRobots):
            xk[0 + ir*n] = posx[centerIds[ir]]
            xk[1 + ir*n] = posy[centerIds[ir]]
            xk[2 + ir*n] = velAgent[centerIds[ir]]
            xk[3 + ir*n] = ort[centerIds[ir]]
        xks[ik] = xk
        Pk = ( np.tile(np.array([sigmaPos**2, sigmaPos**2, (0.05)**2, (0.01)**2]),[1,nRobots]))
        Pks[ik] = np.diag(Pk.squeeze())

    acomArr = np.zeros((nAgents,1))
    ortdotArr = np.zeros((nAgents,1))

    #ftemp = dict()
    #qtemp = dict()
    fs = [np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4))]
    Qks = [np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4)),np.zeros((4,4))]

    # crazyswarm definitions
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    cfs = dict()
    for i in range(0, nAgents):
        cfs[i] = swarm.allcfs.crazyflies[i]

    t = 0.0

    timeHelper.sleep(1.0)

    allcfs = swarm.allcfs #create a variable to represent all swarm members
    allcfs.takeoff(targetHeight=0.75, duration=TAKEOFF_DURATION) #broadcast the takeoff command to all members
    timeHelper.sleep(TAKEOFF_DURATION+1.0) #added additional 1 second of hovering after take off


    
    for i in range(0, nAgents):
        cfs[i].goTo(goal=[float(posx[i]), float(posy[i]), 0.75], yaw=0, duration=5.0)
        #timeHelper.sleep(0.2)
    timeHelper.sleep(5.0+1.0)

    #harekete baslat sonra kalman.
    for i in range(0, nAgents):
        cfs[i].cmdVelocityWorld([float(velAgent[i]*np.cos(ort[i])), float(velAgent[i]*np.sin(ort[i])), 0.0], yawRate=0)
    timeHelper.sleep(0.2)
            

    t0 = timeHelper.time()
    count = 0
    while t < tsim:

        t1 = timeHelper.time()
        t1x = time.time()
        t = t1 - t0

        # get posn meas
        for i in range(0, nAgents):
            posx[i] = (cfs[i].position()[0])
            posy[i] = (cfs[i].position()[1])
            posz[i] = (cfs[i].position()[2])

        #print(posy)
        #####EXCEPTIONS.
        # closer than threshold.
        for i in range(0, nAgents - 1):
            for j in range(i + 1, nAgents):
                dist_ij = np.sqrt((posx[i] - posx[j]) ** 2 + (posy[i] - posy[j]) ** 2)
                if dist_ij < distThreshold:
                    exceptionCase = 1
                    break

        #print(posx)
        #print(posy)
        #print("")
        for i in range(0, nAgents):
            if np.abs(posx[i]) > arenaHardLimit or np.abs(posy[i]) > arenaHardLimit:
                exceptionCase = 2
                break

        # pressed p
        """
        if keyboard.is_pressed('p'):
        	exceptionCase = 3
		"""

        if exceptionCase > 0:
            break


        # commands

        velAgent = velAgent + acomArr * ts *1

        for i in range(0,nAgents):
            if velAgent[i] > vel*2:
                velAgent[i] = vel*2
            elif velAgent[i] < -vel*2:
                velAgent[i] = -vel*2

        ort = ort + ortdotArr * ts * 1
        #print(ort)

        cos_ort = np.cos(ort)
        sin_ort = np.sin(ort)
        velx = velAgent * cos_ort
        vely = velAgent * sin_ort
        velz = 1.5*(0.75-posz)

        for i in range(0, nAgents):
            cfs[i].cmdVelocityWorld([float(velx[i]), float(vely[i]), float(velz[i])], yawRate=0)
            #cfs[i].cmdVelocityWorld([0.0, float(vel), float(velz[i])], yawRate=0)

        # EKF for all agents.
        for ik in range(0, nAgents):
            nRobots = nRobotsArr[ik]
            N = nRobots * n
            xk = xks[ik]
            P_k = Pks[ik]
            centerIds = centerIdsArr[ik]
            xr = np.zeros(shape=(nRobots * n, 1))
            xpre = np.zeros(shape=(nRobots * n, 1))

            for ir in range(0, nRobots):
                xr[0 + ir * n] = posx[centerIds[ir]]
                xr[1 + ir * n] = posy[centerIds[ir]]
                xr[2 + ir * n] = velAgent[centerIds[ir]]
                xr[3 + ir * n] = ort[centerIds[ir]]

            for ir in range(0, nRobots):
                f1_3 = np.cos(xk[3 + ir * n])
                f1_4 = -xk[2 + ir * n] * np.sin(xk[3 + ir * n])
                f2_3 = np.sin(xk[3 + ir * n])
                f2_4 = xk[2 + ir * n] * np.cos(xk[3 + ir * n])

                temp = np.zeros(shape=(n, n))
                temp[0,2] = f1_3
                temp[0,3] = f1_4
                temp[1,2] = f2_3
                temp[1,3] = f2_4

                fs[ir] = temp

                #print(temp)
                if ir == 0:
                    q3t = q3;
                    q4t = q4;
                else:
                    q3t = q3o;
                    q4t = q4o;

                temp2 = np.zeros(shape=(n, n))
                temp2[0, 0] = ((q3t*f1_3**2)/3 + (q4t*f1_4**2)/3)*ts3 + q1*ts
                temp2[0, 1] = (ts3*(f1_3*f2_3*q3t + f1_4*f2_4*q4t))/3
                temp2[0, 2] = (f1_3*q3t*ts2)/2
                temp2[0, 3] = (f1_4*q4t*ts2)/2
                temp2[1, 0] = (ts3*(f1_3*f2_3*q3t + f1_4*f2_4*q4t))/3
                temp2[1, 1] = ((q3t*f2_3**2)/3 + (q4t*f2_4**2)/3)*ts3 + q1*ts
                temp2[1, 2] = (f2_3*q3t*ts2)/2
                temp2[1, 3] = (f2_4*q4t*ts2)/2
                temp2[2, 0] = (f1_3*q3t*ts2)/2
                temp2[2, 1] = (f2_3*q3t*ts2)/2
                temp2[2, 2] = q3t*ts
                temp2[2, 3] = 0
                temp2[3, 0] = (f1_4*q4t*ts2)/2
                temp2[3, 1] = (f2_4*q4t*ts2)/2
                temp2[3, 2] = 0
                temp2[3, 3] = q4t * ts

                Qks[ir] = temp2

            if nRobots == 4:
                H = H3
                Rk = Rk3
                meas = np.dot(H, xr)
                F = sli.block_diag(fs[0], fs[1], fs[2], fs[3])
                Qk = sli.block_diag(Qks[0], Qks[1], Qks[2], Qks[3])
            elif nRobots == 5:
                H = H4
                Rk = Rk4
                meas = np.dot(H, xr)
                F = sli.block_diag(fs[0], fs[1], fs[2], fs[3], fs[4])
                Qk = sli.block_diag(Qks[0], Qks[1], Qks[2], Qks[3], Qks[4])
            elif nRobots == 6:
                H = H5
                Rk = Rk5
                meas = np.dot(H, xr)
                F = sli.block_diag(fs[0], fs[1], fs[2], fs[3], fs[4], fs[5])
                Qk = sli.block_diag(Qks[0], Qks[1], Qks[2], Qks[3], Qks[4], Qks[5])
            else:  # nRobots == 7:
                H = H6
                Rk = Rk6
                meas = np.dot(H, xr)
                F = sli.block_diag(fs[0], fs[1], fs[2], fs[3], fs[4], fs[5], fs[6])
                Qk = sli.block_diag(Qks[0], Qks[1], Qks[2], Qks[3], Qks[4], Qks[5], Qks[6])


            # meas ek noise.
            if addNoise == 1:
                for j in range(0, (nRobots * 2 + 1)):
                    if j == 2:
                        meas[j] = meas[j] + sigmaOrt * np.random.normal(0, 1)
                    else:
                        meas[j] = meas[j] + sigmaPos * np.random.normal(0, 1)


            phiK = np.identity(N) + F * ts

            # state propagate
            for ir in range(0, nRobots):
                xpre[0 + ir * n] = xk[0 + ir * n] + xk[2 + ir * n] * np.cos(xk[3 + ir * n]) * ts
                xpre[1 + ir * n] = xk[1 + ir * n] + xk[2 + ir * n] * np.sin(xk[3 + ir * n]) * ts
                # if ir == centerAgent:
                if ir == 0:
                    # xpre[2 + ir * n] = xk[2 + ir * n] + ts * Kacc * (vel + fparallel_model * alpha - xk[2 + ir * n])
                    # xpre[3 + ir * n] = xk[3 + ir * n] + ts * beta * fperp_model
                    xpre[2 + ir * n] = xk[2 + ir * n] + acomArr[ik] * ts
                    xpre[3 + ir * n] = xk[3 + ir * n] + ortdotArr[ik] * ts
                else:
                    xpre[2 + ir * n] = xk[2 + ir * n]
                    xpre[3 + ir * n] = xk[3 + ir * n]

            # covariance propagation
            """
            Mk = np.dot(phiK, np.dot(P_k, np.transpose(phiK))) + Qk

            # calculate Kalman Gains
            K = np.dot(Mk, np.dot(np.transpose(H), inv(np.dot(H, np.dot(Mk, np.transpose(H))) + Rk)))

            xk = xpre + np.matmul(K, (meas - H.dot(xpre)))

            P_k = np.matmul(np.identity(N) - np.dot(K, H), Mk)
            """
            
            Mk = np.matmul(phiK, np.matmul(P_k, phiK.T)) + Qk
        
            K = np.matmul(Mk.dot(H.T), np.linalg.inv( H.dot(Mk.dot(H.T)) + Rk ))
        
            xk = xpre + np.matmul(K, (meas - H.dot(xpre)))
        
            P_k = np.matmul(np.identity(N)-K.dot(H), Mk)

            # save the state of the Kalman for this robot.
            xks[ik] = xk
            Pks[ik] = P_k

            # calculate the command.
            x1 = xk[0]
            y1 = xk[1]
            v1 = xk[2]
            th1 = xk[3]

            costh1 = np.cos(th1)
            sinth1 = np.sin(th1)

            u1 = np.array([costh1, sinth1])
            u1p = np.array([-sinth1, costh1])

            fparallel = 0
            fperp = 0

            for ir in range(1, nRobots):

                xf = xk[0 + ir * n]
                yf = xk[1 + ir * n]
                vf = xk[2 + ir * n]
                thf = xk[3 + ir * n]

                df = np.sqrt((xf - x1) ** 2 + (yf - y1) ** 2)

                costhf = np.cos(thf)
                sinthf = np.sin(thf)

                uLink = np.array([xf-x1, yf-y1]) / df
                un = np.array([costhf, sinthf])
                unp = np.array([-sinthf, costhf])
            
                u1ulink = np.sum(u1*uLink)
                u1pulink = np.sum(u1p*uLink)
                unulink = np.sum(un*uLink)
                unpulink = np.sum(unp*uLink)

                if centerIds[ir] == iFocal or ik==iFocal:  #bu bağlantı focal ile ise ya da bağlantıyı yapan focal ise.
                    # pentagonun ortasindaki ajan dogal uzunlugu 0.85.
                    fparallel = fparallel + Ksp * (df - dn * 0.85) * u1ulink + b * (
                                vf * unulink - v1 * u1ulink) * u1ulink
                    fperp = fperp + Ksp * (df - dn * 0.85) * u1pulink + b * (vf * unulink - v1 * u1ulink) * u1pulink
                else:
                    fparallel = fparallel + Ksp * (df - dn) * u1ulink + b * (vf * unulink - v1 * u1ulink) * u1ulink
                    fperp = fperp + Ksp * (df - dn) * u1pulink + b * (vf * unulink - v1 * u1ulink) * u1pulink

            #obs, circular
            agentFxObs = 0
            agentFyObs = 0
            for j in range(0,2):
                xObs = x1 - obsX*(j*2-1)
                yObs = y1 - obsY
                rObs = np.sqrt(xObs**2 + yObs**2)
                fObs = 0
                if rObs < (obsR+obsD):
                    fObs = obsK / (rObs - obsR)**2
                    if fObs > maxFobs:
                        fObs = maxFobs

                agentFxObs = agentFxObs + fObs*xObs/rObs
                agentFyObs = agentFyObs + fObs*yObs/rObs


            fparallel_obs = np.sum( np.array([agentFxObs, agentFyObs])*u1 )
            fperp_obs = np.sum( np.array([agentFxObs, agentFyObs])*u1p )

            fparallel = fparallel + fparallel_obs
            fperp = fperp + fperp_obs

            acomArr[ik] = Kacc * (vel + fparallel * alpha - v1)
            ortdotArr[ik] = beta * fperp

            """
            if ik == iFocal:
                print('time')
                print(t)
                print(fs[0])
                print('\n')
            """

        t2 = time.time()

        #log
        np.savetxt(fh,posx, fmt='%.4f',delimiter='\t',newline='\t')
        np.savetxt(fh,posy, fmt='%.4f',delimiter='\t',newline='\t')
        np.savetxt(fh,ort, fmt='%.4f',delimiter='\t',newline='\t')
        np.savetxt(fh,velAgent, fmt='%.4f',delimiter='\t',newline='\t')
        np.savetxt(fh,xks[iFocal], fmt='%.4f',delimiter='\t',newline='\t')
        pkdiag = np.sqrt(np.diag(Pks[iFocal]).squeeze())
        np.savetxt(fh,pkdiag, fmt='%.6f',delimiter='\t',newline='\t')
        #np.savetxt(fh,measSave, fmt='%.4f',delimiter='\t',newline='\t')
        fh.write("\n")

        timeHelper.sleepForRate(sleepRate)


    fh.close()
    print(exceptionCase)

    for i in range(0, nAgents):
        #cfs[i].cmdVelocityWorld([float(velx[i]), float(vely[i]), float(velz[i])], yawRate=0)
        cfs[i].cmdVelocityWorld([0.0, 0.0, 0.0], yawRate=0)

    ### switch from low level to high level commander
    for i in range(len(allcfs.crazyflies)):
        allcfs.crazyflies[i].notifySetpointsStop() #print("notify setpoint stop")

    allcfs.land(targetHeight=0.1, duration=4.0) #land all members using broadcast command
    timeHelper.sleep(4.5) #sleep for landing

if __name__ == "__main__":
    main()
