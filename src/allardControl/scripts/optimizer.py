#!/usr/bin/env python3
from NiaPy.task import StoppingTask, OptimizationType
from NiaPy.benchmarks import Benchmark
from NiaPy.algorithms.basic import GreyWolfOptimizer, ArtificialBeeColonyAlgorithm, CuckooSearch, DynamicFireworksAlgorithmGauss, DynamicFireworksAlgorithm, ParticleSwarmAlgorithm, KrillHerdV4, MutatedParticleSwarmOptimization, CoralReefsOptimization
from NiaPy import Runner
import numpy as np
import time
import sys

# our custom benchmark class
class MyBenchmark(Benchmark):
    def __init__(self, nextPos, currentPos=[0,0,0,0,0,0]):
        self.matWeights = np.array([[1,1,1,1],
                                    [1,1,1,1],
                                    [1,1,1,1],
                                    [0,0,0,0]])
        self.matWeights = self.matWeights / self.matWeights.max()
        self.Rot_z = np.identity(4)
        self.Rot_x = np.identity(4)
        # List type of D-H parameter
        # Do not remove these
        self.d = np.array([0.1273,0,0,0.163941,0.1157,0.0922]) # unit: mm
        self.a = np.array([0,-0.612,-0.5723,0,0,0]) # unit: mm
        self.alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]) # unit: radian
        self.currentPos = currentPos
        self.currentEF = self.calculateEF(self.currentPos)
        self.lowerLimit = np.array([[-4, -np.pi, -np.pi,-2*np.pi,-np.pi,-2*np.pi] for i in range(len(nextPos))]).flatten()
        self.upperLimit = np.array([[4, -np.pi/4, np.pi, 2*np.pi, 2*np.pi, 2*np.pi] for i in range(len(nextPos))]).flatten()
        Benchmark.__init__(self,
                           self.lowerLimit,
                           self.upperLimit)
        self.nextPos = nextPos

    def function(self):
        def evaluate(D, sol):
            val = self.combinemat(self.calculateEF(sol))
            return val
        return evaluate

    def combinemat(self, otherMat):
        ers = []
        # print(self.nextPos)
        for i in range(len(self.nextPos)):
            ers.append(((abs(self.nextPos[i] - otherMat[i])*self.matWeights)).sum()*5)
        return np.sqrt(sum(np.array(ers)**2))
    #+((abs(self.currentEF - otherMat)*self.matWeights)).sum()/30

    def calculateEF(self, angs):
        rets = []
        for j in range(len(angs)//6):
            ret = np.identity(4)
        
            for i in range(6):
                ret = np.matmul(ret, self.HTM(self.d[i], self.a[i], self.alpha[i], angs[j*6+i]))
            # if not(angs[j*6+1] < np.pi and angs[j*6+1] >= 0):
            #     ret *= (angs[j*6+1]*3)
            ret[(abs(ret)-sys.float_info.epsilon) <= sys.float_info.epsilon] = 0
            rets.append(ret)
        return rets

    def HTM(self, d,a,alpha, theta):


        self.Rot_z[0, 0] = self.Rot_z[1, 1] = np.cos(theta)
        self.Rot_z[0, 1] = -np.sin(theta)
        self.Rot_z[1, 0] = np.sin(theta)


        self.Rot_z[2, 3] = d
        self.Rot_x[0, 3] = a

        self.Rot_x[1, 1] = self.Rot_x[2, 2] = np.cos(alpha)
        self.Rot_x[1, 2] = -np.sin(alpha)
        self.Rot_x[2, 1] = np.sin(alpha)

        A_i = np.matmul(self.Rot_z, self.Rot_x)
        return A_i



class trajectoryOptim:

    def __init__(self, currentJoints, newPos):
        self._numSteps = 1
        self._jtAngs = currentJoints
        if self._jtAngs is None:
            self._jtAngs = [0 for i in range(6)]
        self.desiredPos = newPos
        self._optimGen = 30000
        self._optimFes = 50000
        self._minimized = 0.03
        self.desiredPos[abs(self.desiredPos) <= sys.float_info.epsilon] = 0
        self.bm = MyBenchmark(self.desiredPos, self._jtAngs)
        self._currentLoc = self.bm.calculateEF(self._jtAngs)
        self.listOfPositions = np.linspace(self._currentLoc, self.desiredPos, self._numSteps+1)[1:]
        self.bm = MyBenchmark(self.listOfPositions, self._jtAngs)
        self.setupStopTask()

    def setupStopTask(self):
        self.stopTask = StoppingTask(nFES=self._optimFes,
                                     D=len(self._jtAngs) * self._numSteps,
                                     nGEN=self._optimGen,
                                     optType=OptimizationType.MINIMIZATION,
                                     benchmark=self.bm,
                                     refValue=self._minimized)

    def optimize(self, optimizer=ParticleSwarmAlgorithm(), iterations=3, verbose=True):
        best = [10]
        ts = []
        ts.append(time.time())
        for i in range(iterations):
            self.setupStopTask()
            currentRun = optimizer.run(self.stopTask)
            if currentRun[-1] < best[-1]:
                best = currentRun
            if verbose:print('Time: {0:.3f}s Ac: {1:.3e}'.format(time.time()-ts[-1], currentRun[-1]))
            ts.append(time.time())
        print(best[0])
        print(self.bm.calculateEF(best[0]))
        return best, ts, self.bm.calculateEF(best[0])

if __name__ =="__main__":
    pos = [0.3, -0.3, 0.95, np.pi/2, np.pi, 0]
    newPos = [[ 4.958e-01, -4.355e-01, -7.512e-01, -5.488e-01],
            [ 5.644e-01, -4.958e-01,  6.599e-01, -7.335e-01],
            [-6.599e-01, -7.512e-01,  0.000e+00, -2.157e-01],
            [ 0.000e+00,  0.000e+00,  0.000e+00,  1.000e+00]]

    tjOp = trajectoryOptim(pos, newPos)

#%%
    ret = tjOp.optimize(MutatedParticleSwarmOptimization())
    
    