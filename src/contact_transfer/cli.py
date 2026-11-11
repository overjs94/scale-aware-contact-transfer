"""Portable CLI for model inspection, function export, and executable demos."""

import argparse
import json
from pathlib import Path

import numpy as np

from .embodiments import catalog, load
from .robot import Robot


def _robot(model, fixed=False):
    if model in catalog():
        return load(model, **({"floating_base": False} if fixed else {}))
    return Robot(model)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("models", help="List offline URDF presets and source revisions")
    inspect = sub.add_parser("inspect", help="Inspect a preset or a custom URDF")
    inspect.add_argument("model")
    inspect.add_argument("--fixed-base", action="store_true")
    export = sub.add_parser("export", help="Export differentiable CasADi functions")
    export.add_argument("model")
    export.add_argument("--output", required=True)
    export.add_argument("--frame")
    export.add_argument("--fixed-base", action="store_true")
    demo = sub.add_parser("demo-refinement", help="Run an analytically checkable robustness problem")
    sim = sub.add_parser("demo-control", help="Run headless MuJoCo computed-torque tracking")
    sim.add_argument("model", nargs="?", default="franka_panda")
    sim.add_argument("--seconds", type=float, default=1.)
    sim.add_argument("--output", default="outputs/control.npz")
    three = sub.add_parser("demo-three-arm", help="Assumed contacts -> 3 keyframes -> optimization -> MuJoCo tools")
    three.add_argument("--scenario", choices=["cartesian", "franka"], default="cartesian")
    three.add_argument("--case")
    three.add_argument("--contacts", help="External contracted-contact predictions JSON")
    three.add_argument("--output", default="outputs/three_arm")
    three.add_argument("--viewer", help="Export actual multi-robot recording to a web source directory")
    assembly = sub.add_parser("demo-table-assembly", help="30 s two-gripper/point Panda tabletop and leg assembly")
    assembly.add_argument("--case")
    assembly.add_argument("--output", default="outputs/table_assembly")
    assembly.add_argument("--viewer")
    args = parser.parse_args(argv)
    if args.command == "models":
        print(json.dumps(catalog(), indent=2))
    elif args.command == "demo-refinement":
        from .contacts import Capability, SurfacePatch, TaskWrench
        from .refinement import refine_contacts
        task = TaskWrench(np.array([0., 0, 0, 0, 0, -9.81]),
                          np.array([[0., 0, 0, 0, 0, 1]]).T, np.eye(1))
        result = refine_contacts([SurfacePatch.fixed([0, 0, 0], [0, 0, -1])],
                                 task, [Capability(normal_max=20)])
        print(json.dumps({"rho": result.rho, "expected_rho": 9.81,
                          "primal_residual": result.primal_residual}, indent=2))
    elif args.command == "demo-control":
        from .control import ComputedTorque
        from .simulation import MujocoRobot
        robot = _robot(args.model)
        if robot.floating_base:
            parser.error("This controller demo is for fixed-base arms; G1 requires whole-body support control.")
        sim = MujocoRobot(robot)
        q0 = robot.neutral()
        # Select an interior posture, including the Panda's strictly negative joint4.
        q0 = np.maximum(robot.lower+.1, np.minimum(robot.upper-.1, q0))
        sim.reset(q0)
        target = np.clip(q0+.03, robot.lower+.05, robot.upper-.05)
        controller = ComputedTorque(robot)
        log = sim.run(controller, lambda t: (target, np.zeros(robot.nv), np.zeros(robot.nv)),
                      int(args.seconds/sim.model.opt.timestep))
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, **log, q_target=target)
        print(json.dumps({"output": str(path.resolve()),
                          "final_position_error": float(np.linalg.norm(sim.state()[0]-target)),
                          "steps": len(log["time"])}, indent=2))
    elif args.command == "demo-table-assembly":
        from .table_assembly import run_table_assembly
        from .assembly_view import export_assembly_view
        plan, trace, report, flags = run_table_assembly(args.output, case_path=args.case)
        if args.viewer: export_assembly_view(plan, trace, report, flags, args.viewer)
        print(json.dumps(report, indent=2))
        if not report["execution_passed"]: raise SystemExit("Table assembly execution rejected; inspect report.")
    elif args.command == "demo-three-arm":
        from .three_arm import run_three_arm
        from .three_arm_view import export_three_arm_view
        case_path = args.case
        if case_path is None and args.scenario == "franka":
            case_path = Path(__file__).parent/"scenarios/three_franka/case.json"
        plan, trace, seal, report = run_three_arm(args.output, case_path=case_path, contacts_path=args.contacts)
        if args.viewer:
            export_three_arm_view(plan, trace, report, args.viewer)
        print(json.dumps(report, indent=2))
        if not (report["tracking_passed"] and report["seal_retained_before_vent"] and report["force_envelopes_passed"] and report["sampled_collision_check_passed"]):
            raise SystemExit("Three-arm physical execution rejected; inspect report and traces.")
    else:
        robot = _robot(args.model, args.fixed_base)
        if args.command == "inspect":
            q = robot.neutral()
            print(json.dumps({"name": robot.tree.name, "nq": robot.nq, "nv": robot.nv,
                              "joints": robot.joint_names, "floating_base": robot.floating_base,
                              "mass_eigenvalue_min": float(np.linalg.eigvalsh(robot.mass(q))[0]),
                              "gravity_norm": float(np.linalg.norm(robot.gravity(q)))}, indent=2))
        else:
            metadata = robot.symbolic().export(args.output,
                         frames=None if args.frame is None else [args.frame])
            print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
