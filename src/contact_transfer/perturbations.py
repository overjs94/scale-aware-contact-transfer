"""Explicit world-frame wrench pulses applied through MuJoCo's force API."""
from dataclasses import dataclass,asdict
import numpy as np

@dataclass(frozen=True)
class WrenchPulse:
    body: str
    start: float
    duration: float
    force_world_N: tuple
    point_body_m: tuple=(0.,0.,0.)
    moment_world_Nm: tuple=(0.,0.,0.)
    def __post_init__(self):
        values=np.r_[self.start,self.duration,self.force_world_N,self.point_body_m,self.moment_world_Nm]
        if not self.body or len(self.force_world_N)!=3 or len(self.point_body_m)!=3 or len(self.moment_world_Nm)!=3 or not np.isfinite(values).all() or self.start<0 or self.duration<=0:raise ValueError('Invalid world wrench pulse')
    def scale(self,t):
        s=(t-self.start)/self.duration
        return float(np.sin(np.pi*s)**2) if 0<=s<=1 else 0.
    def to_dict(self):return {**asdict(self),'profile':'sin_squared','linear_impulse_Ns':(np.asarray(self.force_world_N)*self.duration/2).tolist(),'frame':'force/moment world; application point body'}

class WrenchSchedule:
    def __init__(self,pulses=()):self.pulses=tuple(pulses);self.ids={}
    def __call__(self,mj,model,data,t):
        for p in self.pulses:
            s=p.scale(t)
            if not s:continue
            if p.body not in self.ids:
                self.ids[p.body]=mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,p.body)
                if self.ids[p.body]<0:raise ValueError('Unknown disturbance body '+p.body)
            bid=self.ids[p.body];point=data.xpos[bid]+data.xmat[bid].reshape(3,3)@p.point_body_m
            mj.mj_applyFT(model,data,s*np.asarray(p.force_world_N),s*np.asarray(p.moment_world_Nm),point,bid,data.qfrc_applied)
    def to_list(self):return [p.to_dict() for p in self.pulses]
