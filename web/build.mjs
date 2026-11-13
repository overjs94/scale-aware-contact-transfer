import { build } from "esbuild";
import { cp, mkdir, rm } from "node:fs/promises";
const here = new URL(".", import.meta.url);
await rm(new URL("dist", here), { recursive: true, force: true });
await mkdir(new URL("dist", here), { recursive: true });
await build({
  entryPoints: [new URL("src/app.js", here).pathname],
  bundle: true, minify: true, format: "esm",
  outfile: new URL("dist/app.js", here).pathname,
});
await build({entryPoints:[new URL('src/comparison.js',here).pathname],bundle:true,minify:true,format:'esm',outfile:new URL('dist/comparison.js',here).pathname});
for (const file of ["index.html", "style.css", "robustness-comparison.html", "comparison.css"])
  await cp(new URL("src/" + file, here), new URL("dist/" + file, here));
await cp(new URL("data", here), new URL("dist/data", here), { recursive: true });
await cp(new URL("models", here), new URL("dist/models", here), { recursive: true });
await cp(new URL("node_modules/three/LICENSE", here), new URL("dist/THREE-LICENSE.txt", here));
await cp(new URL("URDF-LOADER-LICENSE.txt", here), new URL("dist/URDF-LOADER-LICENSE.txt", here));
console.log("Built contact-transfer viewer.");
