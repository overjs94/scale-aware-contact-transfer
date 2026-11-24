import * as THREE from 'three';
import {RoomEnvironment} from 'three/addons/environments/RoomEnvironment.js';

// Authored procedural wood: no downloaded textures or untracked asset license.
function woodTexture(){
  const canvas=document.createElement('canvas');canvas.width=512;canvas.height=512;const c=canvas.getContext('2d');c.fillStyle='#b58550';c.fillRect(0,0,512,512);let seed=71;
  const random=()=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296;};
  for(let i=0;i<360;i++){const y=random()*512;c.strokeStyle=`rgba(${60+Math.round(random()*65)},47,24,${.035+random()*.12})`;c.lineWidth=.3+random()*1.5;c.beginPath();c.moveTo(0,y);for(let x=0;x<=512;x+=16)c.lineTo(x,y+Math.sin(x/100+i)*(.5+random()*2));c.stroke();}
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;texture.wrapS=texture.wrapT=THREE.RepeatWrapping;texture.repeat.set(2,1);return texture;
}

export function createStudio(scene,renderer,sun,rim,canvasFallback){
  const floor=new THREE.Mesh(new THREE.PlaneGeometry(50,50),new THREE.MeshStandardMaterial({color:0xaeb8c1,roughness:.91,metalness:.03}));floor.position.z=-.006;floor.receiveShadow=true;scene.add(floor);
  let environment;const wood=woodTexture();
  if(!canvasFallback){renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;sun.castShadow=true;sun.shadow.mapSize.set(1024,1024);Object.assign(sun.shadow.camera,{left:-4,right:4,top:4,bottom:-4,near:.1,far:20});sun.shadow.bias=-.0001;sun.shadow.normalBias=.004;
    const generator=new THREE.PMREMGenerator(renderer);const room=new RoomEnvironment();environment=generator.fromScene(room,.04).texture;room.dispose();generator.dispose();}
  return {
    mode:'studio',
    set(mode,grid){this.mode=mode;const studio=mode==='studio';floor.visible=studio;if(grid)grid.visible=!studio;scene.background=new THREE.Color(studio?0xdce3e9:0x0c1423);scene.fog=studio?new THREE.Fog(0xdce3e9,15,45):new THREE.FogExp2(0x0c1423,.06);scene.environment=studio?environment:null;scene.environmentIntensity=.7;sun.intensity=studio?2.3:3.5;rim.intensity=studio?.4:1.8;renderer.toneMappingExposure=studio?.9:1.25;},
    materialize(robot,role,mode){
      robot.traverse(node=>{
        if(!node.isMesh)return;node.castShadow=true;node.receiveShadow=true;
        if(mode==='collision')return;
        const materials=Array.isArray(node.material)?node.material:[node.material];node.material=materials.map(old=>{
          if(!old)return old;if(old.userData?.studio)return old;
          const name=(old.name||'').toLowerCase();const isWood=name.includes('wood')||role==='free_furniture';const light=(old.color?.r||0)+(old.color?.g||0)+(old.color?.b||0);
          const m=isWood?new THREE.MeshPhysicalMaterial({color:0xffffff,map:wood,roughness:.48,clearcoat:.18,clearcoatRoughness:.6}):new THREE.MeshStandardMaterial({color:old.color?.clone()||new THREE.Color(.8,.8,.8),map:old.map||null,roughness:light<.2?.75:.36,metalness:name.includes('metal')?.65:light<.2?.05:.14,transparent:old.transparent,opacity:old.opacity,side:old.side});
          m.name=old.name;m.userData.studio=true;return m;
        });if(!Array.isArray(node.material)||node.material.length===1)node.material=node.material[0];
      });
    }
  };
}
