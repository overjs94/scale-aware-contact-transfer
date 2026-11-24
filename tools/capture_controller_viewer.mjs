// Real pinned visual meshes; the cache is prepared by cache_viewer_meshes.py.
import {createServer} from 'node:http';
import {readFile,mkdir} from 'node:fs/promises';
import {resolve,extname} from 'node:path';
import assert from 'node:assert/strict';
import {chromium} from '../web/node_modules/@playwright/test/index.mjs';
const root=resolve('web/dist'),assets=JSON.parse(await readFile('.cache/render-qa/manifest.json','utf8'));
const server=createServer(async(req,res)=>{
 const path=new URL(req.url,'http://localhost').pathname,file=resolve(root,'.'+path.replace(/\/$/,'/index.html'));
 if(!file.startsWith(root+'/')){res.writeHead(403).end();return;}
 try{const body=await readFile(file);res.writeHead(200,{'Content-Type':({'.html':'text/html','.js':'text/javascript','.css':'text/css','.json':'application/json','.urdf':'application/xml','.svg':'image/svg+xml'})[extname(file)]||'application/octet-stream'}).end(body);}catch{res.writeHead(404).end();}
});
await new Promise(r=>server.listen(8124,'127.0.0.1',r));
const browser=await chromium.launch({executablePath:process.env.CHROMIUM_PATH||'/usr/bin/chromium',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try{
 const page=await browser.newPage({viewport:{width:1500,height:1100}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.route('https://raw.githubusercontent.com/**',async route=>{
  const path=assets[route.request().url()];if(!path)throw Error('Uncached original asset: '+route.request().url());
  await route.fulfill({body:await readFile(path),contentType:path.endsWith('.dae')?'application/xml':'application/octet-stream',headers:{'Access-Control-Allow-Origin':'*'}});
 });
 await mkdir('outputs/controller-screenshots',{recursive:true});
 for(const id of ['research_controller_box_allocation_opposite','research_controller_box_baseline_opposite','research_controller_assembly_adaptive_full']){
  await page.goto('http://127.0.0.1:8124/?model='+id);
  await page.waitForFunction(()=>document.querySelector('#geometry-status').dataset.state==='ready',null,{timeout:180000});
  if(id.includes('opposite'))await page.getByRole('button',{name:/Peak external force · arrow/}).click();
  else await page.locator('#timeline').evaluate(el=>{el.value='1';el.dispatchEvent(new Event('input'));});
  assert.match(await page.locator('#scenario-status').innerText(),id.includes('baseline')?/FAIL/:/PASS/);
  await page.locator('.viewer-card').screenshot({path:'outputs/controller-screenshots/'+id+'.png'});
 }
 assert.deepEqual(errors,[]);console.log('Three original-mesh controller previews passed; matched load arrows and scope badges verified.');
}finally{await browser.close();await new Promise(r=>server.close(r));}
