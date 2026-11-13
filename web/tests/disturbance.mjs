import assert from 'node:assert/strict';
import fs from 'node:fs';
import {disturbanceScale} from '../src/disturbance.js';
const hold={start:4,duration:5,ramp_duration_s:.5,profile:'smooth_hold'};
assert.equal(disturbanceScale(hold,5),1);
assert.equal(disturbanceScale(hold,8),1);
assert.equal(disturbanceScale(hold,9.1),0);
const index=JSON.parse(fs.readFileSync(new URL('../data/index.json',import.meta.url)));let checked=0;
for(const row of index.models.filter(r=>/^margin_(hold|platform|compact)_/.test(r.id))){
 const record=JSON.parse(fs.readFileSync(new URL(`../data/${row.id}.json`,import.meta.url))),a=record.report.margin_hold.response;
 a.time_s.forEach((time,i)=>{
  const force=[0,0,0];for(const pulse of record.external_disturbances)for(let axis=0;axis<3;axis++)force[axis]+=pulse.force_world_N[axis]*disturbanceScale(pulse,time);
  force.forEach((v,axis)=>assert.ok(Math.abs(v-a.force_world_N[i][axis])<1e-10,`${row.id}: rendered/native force differs at ${time}`));
 });checked++;
}
console.log(`Visualized force profiles match native schedule samples in ${checked} recordings.`);
