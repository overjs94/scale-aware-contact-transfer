import { build } from "esbuild";
await build({ entryPoints: ["src/urdf-view.js"], outfile: ".test/module.js", bundle: true, format: "esm" });
