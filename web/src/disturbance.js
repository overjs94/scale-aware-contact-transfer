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

export function activeWrenches(pulses,time){
 return pulses.map(pulse=>{
  const scale=disturbanceScale(pulse,time);
  return {pulse,force:(pulse.force_world_N||[0,0,0]).map(v=>v*scale),moment:(pulse.moment_world_Nm||[0,0,0]).map(v=>v*scale)};
 }).filter(w=>Math.hypot(...w.force)>1e-6||Math.hypot(...w.moment)>1e-6);
}

export function disturbedObject(objects,body){
 const exact=objects.find(o=>o.native_body===body);
 if(exact)return exact;
 // Older recordings omit native_body, but preserve unique link IDs.
 const legacy=objects.filter(o=>!o.native_body&&body.endsWith('_'+o.id));
 return legacy.length===1?legacy[0]:objects.length===1?objects[0]:null;
}
