import { createServer } from "node:http";
import { readFile } from "node:fs/promises";
import { resolve, extname } from "node:path";
const root = resolve("dist");
const types = { ".html": "text/html", ".js": "text/javascript", ".json": "application/json", ".css": "text/css", ".urdf": "application/xml" };
createServer(async (req, res) => {
  const path = new URL(req.url, "http://localhost").pathname;
  const file = path === "/test-module.js" ? resolve(".test/module.js") : resolve(root, "."+decodeURIComponent(path).replace(/\/$/, "/index.html"));
  if (path !== "/test-module.js" && !file.startsWith(root+"/")) { res.writeHead(403).end(); return; }
  try { const body = await readFile(file); res.writeHead(200, { "Content-Type": types[extname(file)] || "application/octet-stream" }).end(body); }
  catch { res.writeHead(404).end(); }
}).listen(8123, "127.0.0.1");
