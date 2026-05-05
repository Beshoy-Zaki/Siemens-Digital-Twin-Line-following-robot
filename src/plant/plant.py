#!/usr/bin/env python3

from __future__ import print_function
import struct, sys, argparse, math, csv, random

sys.path.append('pythonGateways/')
import VsiCommonPythonApi  as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23
V   = 1.0
DT  = 0.001

def get_reference_y(x, path_type):
    if path_type == 'straight':
        return 0.0
    elif path_type == 'curved':
        # Sinusoidal path: y_ref = 0.5 * sin(0.2 * x)
        return 0.5 * math.sin(0.2 * x)
    return 0.0

class Plant:
    def __init__(self, args):
        self.componentId = 0
        self.localHost   = args.server_url
        self.domain      = args.domain
        self.portNum     = 50101
        self.noise_std   = args.noise
        self.path_type   = args.path

        self.x     =  0.0
        self.y     =  0.5
        self.theta =  0.1
        self.omega =  0.0

        self.simulationStep      = 0
        self.totalSimulationTime = 0

    def _pack(self, v):   return struct.pack('=d', v)
    def _unpack(self, d): return struct.unpack('=d', d[:8])[0]

    def sendSignal(self, can_id, value):
        vsiCanPythonGateway.setCanId(can_id)
        vsiCanPythonGateway.setDataLengthInBits(64)
        vsiCanPythonGateway.setCanPayloadBits(self._pack(value), 0, 64)
        vsiCanPythonGateway.sendCanPacket()

    def recvSignal(self, can_id):
        data = vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id)
        return self._unpack(data)

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStep      = vsiCommonPythonApi.getSimulationStep()

    def updateKinematics(self):
        noise = random.gauss(0, self.noise_std) if self.noise_std > 0 else 0.0
        self.x     += V * math.cos(self.theta) * DT
        self.y     += (V * math.sin(self.theta) + noise) * DT
        self.theta += self.omega * DT
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(
            self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)

        with open('trajectory_data.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_ns','x','y','theta','omega','y_ref','error'])

            try:
                vsiCommonPythonApi.waitForReset()
                self.updateInternalVariables()
                if self.totalSimulationTime == 0:
                    self.totalSimulationTime = 60_000_000_000

                nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

                while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                    self.updateInternalVariables()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception("stopRequested")

                    y_ref = get_reference_y(self.x, self.path_type)

                    self.sendSignal(CAN_X,     self.x)
                    self.sendSignal(CAN_Y,     self.y - y_ref)  # send error from path
                    self.sendSignal(CAN_THETA, self.theta)

                    self.omega = self.recvSignal(CAN_OMEGA)
                    self.updateKinematics()

                    t   = vsiCommonPythonApi.getSimulationTimeInNs()
                    err = self.y - y_ref
                    w.writerow([t, self.x, self.y, self.theta, self.omega, y_ref, err])
                    print(f"[PLANT] t={t}ns  x={self.x:.3f}  y={self.y:.4f}"
                          f"  y_ref={y_ref:.4f}  err={err:.4f}  ω={self.omega:.4f}")

                    self.updateInternalVariables()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception("stopRequested")
                    nextExpectedTime += self.simulationStep
                    now = vsiCommonPythonApi.getSimulationTimeInNs()
                    if now >= nextExpectedTime:
                        continue
                    if nextExpectedTime > self.totalSimulationTime:
                        vsiCommonPythonApi.advanceSimulation(self.totalSimulationTime - now)
                        break
                    vsiCommonPythonApi.advanceSimulation(nextExpectedTime - now)

            except Exception as e:
                if str(e) == "stopRequested":
                    print("[PLANT] Stop requested.")
                    vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
                else:
                    print(f"[PLANT] ERROR: {e}")
                    raise
            except:
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

        print("[PLANT] Done.")


def main():
    p = argparse.ArgumentParser("Plant")
    p.add_argument('--domain',     default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--noise', type=float, default=0.0)
    p.add_argument('--path', default='straight', choices=['straight','curved'])
    Plant(p.parse_args()).mainThread()

if __name__ == '__main__':
    main()
