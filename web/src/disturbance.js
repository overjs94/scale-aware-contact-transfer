// Keep visualized force identical to the force profile passed to MuJoCo.
export function disturbanceScale(pulse,time){
 const elapsed=time-pulse.start;
 if(elapsed<0||elapsed>pulse.duration)return 0;
 if(pulse.profile==='smooth_hold'){
  const edge=Math.min(elapsed,pulse.duration-elapsed);
  return edge<pulse.ramp_duration_s?Math.sin(.5*Math.PI*edge/pulse.ramp_duration_s)**2:1;
 }
 if(pulse.profile!==undefined&&pulse.profile!=='sin_squared')throw Error('Unknown disturbance profile');
 return Math.sin(Math.PI*elapsed/pulse.duration)**2;
}
