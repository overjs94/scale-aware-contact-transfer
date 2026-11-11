"""Assumed network contacts -> three keyframes -> two CITO solves -> MuJoCo."""

import argparse
import json

from contact_transfer.three_arm import run_three_arm
from contact_transfer.three_arm_view import export_three_arm_view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", help="Contracted-contact scenario JSON; defaults to the packaged testbed")
    parser.add_argument("--contacts", help="External contracted-contact predictions JSON")
    parser.add_argument("--output", default="outputs/three_arm")
    parser.add_argument("--viewer", help="Optionally export actual replay into this web source directory")
    args = parser.parse_args()
    plan, trace, seal, report = run_three_arm(args.output, case_path=args.case, contacts_path=args.contacts)
    if args.viewer:
        export_three_arm_view(plan, trace, report, args.viewer)
    print(json.dumps(report, indent=2))
    if not (report["tracking_passed"] and report["seal_retained_before_vent"] and report["force_envelopes_passed"]):
        raise SystemExit("Three-arm dynamic execution validation failed; inspect report and traces.")


if __name__ == "__main__":
    main()
