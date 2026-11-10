"""Simulation-ready controllers and position/velocity command conversion.

This module does not open a hardware connection. Actual servo transport and
device-specific watchdogs belong to the robot-side adapter.
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .lie import rotation_error, so3_exp


class ComputedTorque:
    def __init__(self, robot, kp=100., kd=20.):
        if robot.floating_base:
            raise ValueError("Floating-base control requires a whole-body constrained controller.")
        self.robot = robot
        self.kp = np.broadcast_to(kp, (robot.nv,))
        self.kd = np.broadcast_to(kd, (robot.nv,))
        if np.any(self.kp < 0) or np.any(self.kd < 0):
            raise ValueError("gains must be nonnegative")

    def command(self, q, v, q_desired, v_desired, a_desired, *, reaction_generalized=None):
        acceleration = (np.asarray(a_desired)+self.kp*(np.asarray(q_desired)-q)
                        +self.kd*(np.asarray(v_desired)-v))
        tau = self.robot.inverse_dynamics(q, v, acceleration)
        if reaction_generalized is not None:
            tau -= np.asarray(reaction_generalized)
        return np.clip(tau, -self.robot.effort, self.robot.effort)


class CartesianAdmittance:
    """Mixed-frame [omega_body; pdot_world] admittance around a reference pose.

    mass/damping/stiffness are positive diagonal virtual parameters. Input
    wrench is object-on-robot [moment_tool;force_world] about the same origin.
    """
    def __init__(self, mass, damping, stiffness):
        self.mass, self.damping, self.stiffness = (
            np.broadcast_to(np.asarray(x, float), (6,)).copy()
            for x in (mass, damping, stiffness))
        if np.any(self.mass <= 0) or np.any(self.damping < 0) or np.any(self.stiffness < 0):
            raise ValueError("invalid admittance parameters")
        self.velocity = np.zeros(6)

    def step(self, pose, reference, measured_wrench, desired_wrench, dt):
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError("dt must be positive and finite")
        e = np.r_[np.asarray(rotation_error(pose[:3, :3], reference[:3, :3])).reshape(3),
                  pose[:3, 3]-reference[:3, 3]]
        force_error = np.asarray(measured_wrench)-np.asarray(desired_wrench)
        # Implicit damping update. Velocity components remain in the current
        # moving tool frame; re-express the angular state after the pose step.
        vnew = (self.mass*self.velocity+dt*(force_error-self.stiffness*e))/(self.mass+dt*self.damping)
        out = np.array(pose, float, copy=True)
        dR = np.asarray(so3_exp(dt*vnew[:3]))
        out[:3, :3] = pose[:3, :3]@dR
        out[:3, 3] += dt*vnew[3:]
        self.velocity = np.r_[dR.T@vnew[:3], vnew[3:]]
        return out, vnew


def position_servo_step(robot, q, frame, desired_twist, dt, *, damping=1e-3):
    """Bounded differential IK converting mixed twist to joint position command."""
    if robot.floating_base or dt <= 0:
        raise ValueError("position servo requires a fixed-base robot and dt>0")
    q = np.asarray(q, float)
    J = robot.jacobian(q, frame)
    lo = np.maximum(-robot.velocity, (robot.lower-q)/dt)
    hi = np.minimum(robot.velocity, (robot.upper-q)/dt)
    if np.any(lo >= hi):
        raise ValueError("current state has no interior feasible servo velocity")
    A = np.vstack((J, damping*np.eye(robot.nv)))
    b = np.r_[np.asarray(desired_twist), np.zeros(robot.nv)]
    solution = lsq_linear(A, b, bounds=(lo, hi), tol=1e-10)
    if not solution.success:
        raise RuntimeError("bounded differential IK did not converge")
    return robot.integrate(q, dt*solution.x), solution.x
