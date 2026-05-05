#!/usr/bin/env python3

from __future__ import print_function
import struct, sys, argparse, math

sys.path.append('pythonGateways/')
import VsiCommonPythonApi  as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23
DT = 0.001

class PID:
    def __init__(self, Kp, Ki, Kd, limit=3.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.limit     = limit
        self.integral  = 0.0
        self.prev_err  = 0.0

    def step(self, ref, meas, dt=DT):
        err            = ref - meas
        self.integral += err * dt
        deriv          = (err - self.prev_err) / dt
        self.prev_err  = err
        raw = self.Kp*err + self.Ki*self.integral + self.Kd*deriv
        return max(-self.limit, min(self.limit, raw))


class Controller:
    def __init__(self, args):
        self.componentId = 1
        self.localHost   = args.server_url
        self.domain      = args.domain
        self.portNum     = 50102

        self.pid = PID(args.Kp, args.Ki, args.Kd)
        self.path_type = args.path
        self.y_ref = 0.0   # will be updated

        self.x, self.y, self.theta, self.omega = 0., 0., 0., 0.
        self.simulationStep      = 0
        self.totalSimulationTime = 0

    def _pack(self, v):   return struct.pack('=d', v)
    def _unpack(self, d): return struct.unpack('=d', d[:8])[0]

    def get_y_ref(self, x):
        if self.path_type == 'straight':
            return 0.0
        elif self.path_type == 'curved':
            # Piecewise circular arcs: rise, fall, then rise again.
            radius = 20.0
            seg = 15.0

            def arc_height(u):
                u = max(0.0, min(seg, u))
                return radius - math.sqrt(max(0.0, radius * radius - u * u))

            if x <= seg:
                return arc_height(x)
            if x <= 2.0 * seg:
                return arc_height(seg) - arc_height(x - seg)
            if x <= 3.0 * seg:
                return arc_height(x - 2.0 * seg)
            return arc_height(seg)
        else:
            return 0.0

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

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(
            self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)

        try:
            vsiCommonPythonApi.waitForReset()
            self.updateInternalVariables()
            if self.totalSimulationTime == 0:
                self.totalSimulationTime = 10_000_000_000

            nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

            while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                self.updateInternalVariables()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception("stopRequested")

                # 1) RECEIVE plant state
                self.x     = self.recvSignal(CAN_X)
                self.y     = self.recvSignal(CAN_Y)
                self.theta = self.recvSignal(CAN_THETA)

                # 2) COMPUTE PID  (control lateral error)
                self.y_ref = self.get_y_ref(self.x)
                self.omega = self.pid.step(self.y_ref, self.y)

                # 3) SEND steering command
                self.sendSignal(CAN_OMEGA, self.omega)

                t = vsiCommonPythonApi.getSimulationTimeInNs()
                print(f"[CTRL]  t={t}ns  y={self.y:.4f}  y_ref={self.y_ref:.4f}"
                      f"  err={self.y_ref-self.y:.4f}  ω={self.omega:.4f}")

                # 4) ADVANCE
                self.updateInternalVariables()
                if vsiCommonPythonApi.isStopRequested():
                    raise Exception("stopRequested")
                nextExpectedTime += self.simulationStep
                now = vsiCommonPythonApi.getSimulationTimeInNs()
                if now >= nextExpectedTime:
                    continue
                if nextExpectedTime > self.totalSimulationTime:
                    vsiCommonPythonApi.advanceSimulation(
                        self.totalSimulationTime - now)
                    break
                vsiCommonPythonApi.advanceSimulation(nextExpectedTime - now)

        except Exception as e:
            if str(e) == "stopRequested":
                print("[CTRL] Stop requested.")
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
            else:
                print(f"[CTRL] ERROR: {e}")
                raise
        except:
            vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)


def main():
    p = argparse.ArgumentParser("Controller")
    p.add_argument('--domain',     default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--Kp', type=float, default=2.0)
    p.add_argument('--Ki', type=float, default=0.1)
    p.add_argument('--Kd', type=float, default=0.5)
    p.add_argument('--path', default='straight', choices=['straight','curved'])
    Controller(p.parse_args()).mainThread()

if __name__ == '__main__':
    main()
