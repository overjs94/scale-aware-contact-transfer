import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {englishRecording,recordingTitle} from '../src/presentation-en.js';
import {libraryEntry} from '../src/library.js';
const index=JSON.parse(await readFile(new URL('../data/index.json',import.meta.url)));
const hangul=/[\uac00-\ud7a3]/;
for(const model of index.models){
 const original=JSON.parse(await readFile(new URL(`../data/${model.id}.json`,import.meta.url)));
 const before=JSON.stringify(original), english=englishRecording(original);
 assert.equal(JSON.stringify(original),before,`${model.id}: original was mutated`);
 for(const key of ['q','positions','objects','times','torques','report','provenance','native_contact_samples'])assert.equal(english[key],original[key],`${model.id}: ${key} must retain its original reference`);
 for(const value of [english.title,...Object.values(english.display||{}),...(english.events||[]).map(e=>e.id),...(english.keyframes||[]).map(e=>e.id),libraryEntry(model).title])if(typeof value==='string')assert.equal(hangul.test(value),false,`${model.id}: untranslated presentation: ${value}`);
 assert.equal(recordingTitle(model.id,model.title),english.title);
}
console.log(`English presentation checked: ${index.models.length} recordings; original measurements and reports unchanged.`);
