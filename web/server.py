"""Serve the viewer and execute new local MuJoCo runs. No robot hardware access."""

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading

from simulate import generate


lock = threading.Lock()


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(Path(__file__).parent/"dist"), **kwargs)

    def respond(self, code, body):
        data=json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type","application/json")
        self.send_header("Content-Length",str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/health":
            return self.respond(200,{"backend":"python_mujoco","ready":True})
        return super().do_GET()

    def do_POST(self):
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
    args=parser.parse_args()
    if not (Path(__file__).parent/"dist"/"index.html").is_file():
        raise SystemExit("Run npm --prefix web install && npm --prefix web run build first.")
    print(f"Serving contact-transfer simulation UI on {args.host}:{args.port}",flush=True)
    ThreadingHTTPServer((args.host,args.port),Handler).serve_forever()
