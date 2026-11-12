"""Serve the viewer and execute new local MuJoCo runs. No robot hardware access."""

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import mimetypes
from urllib.parse import urlsplit, unquote, parse_qs
from experiments import Jobs, PRESETS, ARTICULATED

from simulate import generate


lock = threading.Lock()
jobs = Jobs()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(Path(__file__).parent/"dist"), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        return super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Allow", "GET, POST, OPTIONS")
        self.end_headers()

    def respond(self, code, body):
        data=json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length",str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/experiments/preset":
            preset=parse_qs(urlsplit(self.path).query).get('preset',['three_franka'])[0]
            if preset in ARTICULATED:return self.respond(200,json.loads((PRESETS/'articulated'/(preset+'.json')).read_text()))
            if preset not in {'three_franka','table_assembly','panel_fixture','mobile_panel'}:return self.respond(400,{'error':'Unknown preset'})
            return self.respond(200,json.loads((PRESETS/preset/"case.json").read_text()))
        if path.startswith("/api/experiments/"):
            try:
                parts = path.split("/")
                root = jobs.path(parts[3])
                if len(parts) == 4: return self.respond(200,jobs.get(parts[3]))
                if parts[4] != "assets": return self.respond(404,{"error":"unknown asset"})
                asset = (root/"viewer"/unquote("/".join(parts[5:]))).resolve()
                if not asset.is_relative_to((root/"viewer").resolve()) or not asset.is_file():
                    return self.respond(404,{"error":"unknown asset"})
                data = asset.read_bytes(); self.send_response(200)
                self.send_header("Content-Type",mimetypes.guess_type(asset.name)[0] or "application/octet-stream")
                self.send_header("Content-Length",str(len(data))); self.end_headers(); self.wfile.write(data)
                return
            except (ValueError,FileNotFoundError) as error: return self.respond(404,{"error":str(error)})
        if self.path == "/api/health":
            meshes = PRESETS.parents[2]/".cache/panda_meshes"
            return self.respond(200,{"backend":"python_mujoco","ready":True,
                                     "experiments_ready": (meshes/"link7.stl").is_file()})
        return super().do_GET()

    def do_POST(self):
        origin=self.headers.get("Origin")
        if origin and urlsplit(origin).netloc != self.headers.get("Host"):
            return self.respond(403,{"error":"Use the same-origin local UI for computation"})
        path=urlsplit(self.path).path
        if path.startswith("/api/experiments"):
            try:
                if path.endswith("/cancel"):
                    return self.respond(200,jobs.cancel(path.split("/")[3]))
                if path != "/api/experiments":return self.respond(404,{"error":"unknown endpoint"})
                size=int(self.headers.get("Content-Length","0"))
                if not 0<size<=32768:raise ValueError("Invalid request size")
                return self.respond(202,jobs.submit(json.loads(self.rfile.read(size))))
            except BlockingIOError as error:return self.respond(409,{"error":str(error)})
            except (ValueError,KeyError,TypeError,FileNotFoundError) as error:return self.respond(400,{"error":str(error)})
        if self.path != "/api/simulate":
            return self.respond(404,{"error":"unknown endpoint"})
        try:
            size=int(self.headers.get("Content-Length","0"))
            if not 0 < size <= 4096:
                return self.respond(400,{"error":"invalid request size"})
            request=json.loads(self.rfile.read(size))
            if request.get("model") == "unitree_g1":
                return self.respond(400,{"error":"G1 requires a whole-body controller; this endpoint runs fixed-base arms."})
            if not lock.acquire(blocking=False):
                return self.respond(409,{"error":"a simulation is already running"})
            try:
                result=generate(request["model"],float(request.get("duration",3)),
                                float(request.get("amplitude",.12)))
            finally:
                lock.release()
            self.respond(200,result)
        except (KeyError,TypeError,ValueError) as error:
            self.respond(400,{"error":str(error)})
        except Exception as error:
            self.respond(500,{"error":type(error).__name__+": "+str(error)})


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host",default="127.0.0.1")
    parser.add_argument("--port",type=int,default=8000)
    parser.add_argument("--strictPort",action="store_true",help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not (Path(__file__).parent/"dist"/"index.html").is_file():
        raise SystemExit("Run npm --prefix web install && npm --prefix web run build first.")
    print(f"Serving contact-transfer simulation UI on {args.host}:{args.port}",flush=True)
    ThreadingHTTPServer((args.host,args.port),Handler).serve_forever()
