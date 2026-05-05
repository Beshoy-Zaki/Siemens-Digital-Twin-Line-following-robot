#!/usr/bin/env python3

from __future__ import print_function
import struct, sys, argparse, csv, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.append('pythonGateways/')
import VsiCommonPythonApi  as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway

CAN_X, CAN_Y, CAN_THETA, CAN_OMEGA = 20, 21, 22, 23

class Visualizer:
    def __init__(self, args):
        self.componentId = 2
        self.localHost   = args.server_url
        self.domain      = args.domain
        self.portNum     = 50103
        self.label       = args.label
        self.path_type   = args.path

        self.x = self.y = self.theta = self.omega = 0.0
        self.simulationStep = self.totalSimulationTime = 0

        self.time_hist  = []
        self.x_hist     = []
        self.y_hist     = []
        self.theta_hist = []
        self.omega_hist = []

        self.max_overshoot = 0.0
        self.steady_errors = []
        self.STEADY_START  = 5_000_000_000

    def _unpack(self, data): return struct.unpack('=d', data[:8])[0]

    def get_y_ref(self, x):
        if self.path_type == 'straight':
            return 0.0
        elif self.path_type == 'curved':
            # Piecewise circular arcs: same reference used by the controller.
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

    def recvSignal(self, can_id):
        return self._unpack(vsiCanPythonGateway.recvVariableFromCanPacket(8, 0, 64, can_id))

    def updateInternalVariables(self):
        self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
        self.simulationStep      = vsiCommonPythonApi.getSimulationStep()

    def save_matplotlib(self):
        label = self.label
        sse   = sum(self.steady_errors)/len(self.steady_errors) if self.steady_errors else 0

        # Compute settling time
        threshold = 0.05  # 5cm
        settling_time = self.time_hist[-1]
        errors_hist = [y - self.get_y_ref(x) for y, x in zip(self.y_hist, self.x_hist)]
        for i in range(len(errors_hist)):
            if all(abs(e) < threshold for e in errors_hist[i:]):
                settling_time = self.time_hist[i]
                break

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'Line-Following Robot — {label}', fontsize=14, fontweight='bold')

        axes[0,0].plot(self.x_hist, self.y_hist, 'b-', linewidth=1, label='Robot path')
        if self.path_type == 'straight':
            axes[0,0].axhline(0, color='r', linestyle='--', linewidth=2, label='Reference y=0')
        else:
            import numpy as np
            x_min, x_max = min(self.x_hist), max(self.x_hist)
            x_path = np.linspace(x_min, x_max, 100)
            y_path = [self.get_y_ref(x) for x in x_path]
            axes[0,0].plot(x_path, y_path, 'r--', linewidth=2, label='Reference path')
        axes[0,0].set_xlabel('x (m)'); axes[0,0].set_ylabel('y (m)')
        axes[0,0].set_title('2D Trajectory'); axes[0,0].legend(); axes[0,0].grid(True)

        errors_hist = [y - self.get_y_ref(x) for y, x in zip(self.y_hist, self.x_hist)]

        axes[0,1].plot(self.time_hist, errors_hist, 'b-', linewidth=1)
        axes[0,1].axhline(0, color='r', linestyle='--', linewidth=2, label='Reference')
        axes[0,1].set_xlabel('Time (s)'); axes[0,1].set_ylabel('Lateral Error y - y_ref (m)')
        axes[0,1].set_title('Lateral Error vs Time'); axes[0,1].legend(); axes[0,1].grid(True)

        axes[1,0].plot(self.time_hist, self.omega_hist, 'g-', linewidth=1)
        axes[1,0].axhline(0, color='r', linestyle='--')
        axes[1,0].set_xlabel('Time (s)'); axes[1,0].set_ylabel('ω (rad/s)')
        axes[1,0].set_title('Steering Command vs Time'); axes[1,0].grid(True)

        axes[1,1].plot(self.time_hist, [math.degrees(t) for t in self.theta_hist], 'm-', linewidth=1)
        axes[1,1].axhline(0, color='r', linestyle='--')
        axes[1,1].set_xlabel('Time (s)'); axes[1,1].set_ylabel('θ (deg)')
        axes[1,1].set_title('Heading Angle vs Time'); axes[1,1].grid(True)

        kpi = (f"Label: {label}\n"
               f"Max Overshoot : {self.max_overshoot:.4f} m\n"
               f"Final Error   : {abs(errors_hist[-1]):.4f} m\n"
               f"Steady-State  : {sse:.4f} m\n"
               f"Settling Time : {settling_time:.1f} s")
        fig.text(0.02, 0.01, kpi, fontsize=9,
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        fname = f'results_{label}.png'
        plt.savefig(fname, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"[VIZ] Saved: {fname}")

    def mainThread(self):
        dSession = vsiCommonPythonApi.connectToServer(
            self.localHost, self.domain, self.portNum, self.componentId)
        vsiCanPythonGateway.initialize(dSession, self.componentId)

        with open('visualizer_log.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_ns','x','y','theta','omega','lateral_error'])
            try:
                vsiCommonPythonApi.waitForReset()
                self.updateInternalVariables()
                if self.totalSimulationTime == 0:
                    self.totalSimulationTime = 30_000_000_000

                nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()

                while vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime:
                    self.updateInternalVariables()
                    if vsiCommonPythonApi.isStopRequested():
                        raise Exception("stopRequested")

                    self.x     = self.recvSignal(CAN_X)
                    self.y     = self.recvSignal(CAN_Y)
                    self.theta = self.recvSignal(CAN_THETA)
                    self.omega = self.recvSignal(CAN_OMEGA)

                    t_ns = vsiCommonPythonApi.getSimulationTimeInNs()
                    y_ref = self.get_y_ref(self.x)
                    err  = self.y - y_ref

                    if abs(err) > self.max_overshoot:
                        self.max_overshoot = abs(err)
                    if t_ns > self.STEADY_START:
                        self.steady_errors.append(abs(err))

                    self.time_hist.append(t_ns * 1e-9)
                    self.x_hist.append(self.x)
                    self.y_hist.append(self.y)
                    self.theta_hist.append(self.theta)
                    self.omega_hist.append(self.omega)

                    w.writerow([t_ns, self.x, self.y, self.theta, self.omega, err])

                    # Print every 100 steps
                    if len(self.time_hist) % 100 == 0:
                        print(f"[VIZ] t={t_ns*1e-9:.1f}s  y={self.y:.4f}  err={err:.4f}")

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
                    print("[VIZ] Stop requested.")
                    vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
                else:
                    print(f"[VIZ] ERROR: {e}")
                    raise
            except:
                vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)

        if len(self.time_hist) > 10:
            self.save_matplotlib()

        sse = sum(self.steady_errors)/len(self.steady_errors) if self.steady_errors else 0
        final_err = 0.0
        if self.y_hist and self.x_hist:
            final_err = abs(self.y_hist[-1] - self.get_y_ref(self.x_hist[-1]))
        print(f"\n=== KPI SUMMARY ({self.label}) ===")
        print(f"Max overshoot    : {self.max_overshoot:.4f} m")
        print(f"Final error      : {final_err:.4f} m")
        print(f"Steady-state err : {sse:.4f} m")
        print(f"Sim duration     : {self.time_hist[-1] if self.time_hist else 0:.1f} s")

def main():
    p = argparse.ArgumentParser("Visualizer")
    p.add_argument('--domain',     default='AF_UNIX')
    p.add_argument('--server-url', default='localhost')
    p.add_argument('--label',      default='Kp2.0_Ki0.1_Kd0.5')
    p.add_argument('--path', default='straight', choices=['straight','curved'])
    Visualizer(p.parse_args()).mainThread()

if __name__ == '__main__':
    main()
