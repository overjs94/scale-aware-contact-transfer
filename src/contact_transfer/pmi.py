"""Passive midpoint step for a constant diagonal virtual mechanical system.

This is a controller-side virtual mass/damper/spring integrator, not a claim that
MuJoCo or CRISP uses PMI. Nonlinear multibody/contact PMI requires its own
discrete energy and reaction treatment.
"""
import numpy as np


def passive_midpoint_step(position,velocity,force,dt,*,mass,damping,stiffness=0.):
    """Power-conjugate displacement uses the midpoint velocity.

    With constant diagonal M,D,K and midpoint-held input f,
    H_next-H = dt*f.T*v_mid - dt*v_mid.T*D*v_mid (up to rounding).
    No saturation, moving-frame projection or nonlinear-potential passivity is
    asserted by this linear-system primitive.
    """
    x,v,f=np.broadcast_arrays(np.asarray(position,float),np.asarray(velocity,float),np.asarray(force,float))
    M,D,K=[np.broadcast_to(np.asarray(a,float),x.shape) for a in [mass,damping,stiffness]]
    if dt<=0 or not np.isfinite(dt) or np.any(M<=0) or np.any(D<0) or np.any(K<0) or not all(np.isfinite(a).all() for a in [x,v,f,M,D,K]):raise ValueError('Finite states, positive mass/timestep and nonnegative damping/stiffness required')
    midpoint=((2*M/dt)*v+f-K*x)/(2*M/dt+D+.5*dt*K)
    xn=x+dt*midpoint;vn=2*midpoint-v
    energy=lambda p,w:float(np.sum(.5*M*w*w+.5*K*p*p))
    supplied=float(dt*np.sum(f*midpoint));dissipated=float(dt*np.sum(D*midpoint*midpoint));change=energy(xn,vn)-energy(x,v)
    return xn,vn,dict(midpoint_velocity=midpoint,energy_change=change,input_work=supplied,dissipated_energy=dissipated,balance_residual=change-supplied+dissipated)
