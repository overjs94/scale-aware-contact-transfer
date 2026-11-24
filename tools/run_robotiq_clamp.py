"""Check native Robotiq mechanics, four-bar actuation and finite jaw force."""
import argparse
import json
from contact_transfer.robotiq import run_native_clamp

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',default='.cache/robotiq_2f85');parser.add_argument('--output',default='outputs/robotiq_2f85');parser.add_argument('--viewer');args=parser.parse_args();report=run_native_clamp(args.assets,args.output);print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(2)
    if args.viewer:
        from contact_transfer.robotiq_view import export_clamp
        export_clamp(args.output,args.viewer)
