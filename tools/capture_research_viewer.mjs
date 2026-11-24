// Real pinned meshes from the integration cache; no synthetic geometry.
import {createServer} from 'node:http';
import {readFile,mkdir} from 'node:fs/promises';
import {resolve,extname} from 'node:path';
import {chromium} from '../web/node_modules/@playwright/test/index.mjs';
const root=resolve('web/dist'),assets=JSON.parse(await readFile('.cache/render-qa/manifest.json','utf8'));
const types={'.html':'text/html','.js':'text/javascript','.css':'text/css','.json':'application/json','.urdf':'application/xml','.svg':'image/svg+xml','.png':'image/png'};
const server=createServer(async(req,res)=>{
 const path=new URL(req.url,'http://localhost').pathname,file=resolve(root,'.'+path.replace(/\/$/,'/index.html'));
 if(!file.startsWith(root+'/')){res.writeHead(403).end();return;}
 try{const body=await readFile(file);res.writeHead(200,{'Content-Type':types[extname(file)]||'application/octet-stream'}).end(body);}catch{res.writeHead(404).end();}
});
await new Promise(r=>server.listen(8123,'127.0.0.1',r));
const browser=await chromium.launch({executablePath:process.env.CHROMIUM_PATH||'/usr/bin/chromium',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
try{
 const page=await browser.newPage({viewport:{width:1500,height:1100}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
 await page.route('https://raw.githubusercontent.com/**',async route=>{
  const path=assets[route.request().url()];if(!path)throw Error('Uncached real asset: '+route.request().url());
  await route.fulfill({body:await readFile(path),contentType:path.endsWith('.dae')?'application/xml':'application/octet-stream',headers:{'Access-Control-Allow-Origin':'*'}});
 });
 await mkdir('outputs/research-study/screenshots',{recursive:true});
 for(const id of ['research_full_assembly_external_push','research_probe_box_on_opposite']){
  await page.goto('http://127.0.0.1:8123/?model='+id);
  await page.waitForFunction(()=>document.querySelector('#geometry-status').dataset.state==='ready',null,{timeout:180000});
  await page.getByRole('button',{name:/Peak external force · arrow/}).click();
  await page.locator('.viewer-card').screenshot({path:'outputs/research-study/screenshots/'+id+'.png'});
 }
 await page.goto('http://127.0.0.1:8123/robustness-comparison.html?study=current');
 await page.locator('#play').waitFor({state:'visible'});
 await page.waitForFunction(()=>!document.querySelector('#play').disabled,null,{timeout:180000});
 await page.locator('#time').evaluate(el=>{el.value='10';el.dispatchEvent(new Event('input'));});
 await page.screenshot({path:'outputs/research-study/screenshots/source-margin-pair.png'});
 if(errors.length)throw Error(errors.join('\n'));
 console.log('Captured three real-mesh previews; no browser exceptions');
}finally{await browser.close();await new Promise(r=>server.close(r));}
