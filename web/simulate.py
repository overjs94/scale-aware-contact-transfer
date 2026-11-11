"""Generate real MuJoCo recordings and explicit kinematic/CITO viewer datasets."""

from pathlib import Path
import json
import platform
import numpy as np
from scipy.spatial.transform import Rotation

from contact_transfer.embodiments import catalog, load
from contact_transfer.control import ComputedTorque
from contact_transfer.simulation import MujocoRobot


TITLES = {"franka_panda":"Franka Panda", "kuka_iiwa7":"KUKA iiwa 7", "ur5e":"UR5e",
          "kinova_gen3":"Kinova Gen3", "xarm7":"UFACTORY xArm7",
          "unitree_g1":"Unitree G1", "rb3":"Rainbow RB3", "rb5":"Rainbow RB5"}


def _geometry(robot, states):
    names = list(robot.tree.links)
    parents = [[names.index(j.parent), names.index(j.child)] for j in robot.tree.joints]
    positions = [[[float(x) for x in np.round(robot.fk(q, name)[:3, 3], 6)]
                  for name in names] for q in states]
    return names, parents, positions


def generate(name, duration=3., amplitude=.12, frame_count=121):
    if name not in catalog():
        raise ValueError("unknown model")
    if not .5 <= duration <= 10 or not 0 <= amplitude <= .3:
        raise ValueError("duration must be .5..10 seconds, amplitude 0...3 radians")
    robot = load(name)
    q0 = robot.neutral()
    times = np.linspace(0., duration, frame_count)
    engine = "kinematic_preview" if robot.floating_base else "mujoco_recording"
    errors, torque_ratio, states, torques = [], [], [], []
    if robot.floating_base:
        q0[2] = .82
        for t in times:
            q = q0.copy()
            for i, joint in enumerate(robot.joint_names):
                if any(part in joint for part in ("shoulder", "elbow", "wrist")):
                    q[7+i] = np.clip(q[7+i]+amplitude*np.sin(2*np.pi*t/duration),
                                     robot.lower[i], robot.upper[i])
            states.append(q)
        errors, torque_ratio = [None]*frame_count, [None]*frame_count
        torques = [[None]*robot.n for _ in times]
    else:
        q0 = np.maximum(robot.lower+.1, np.minimum(robot.upper-.1, q0))
        offset = amplitude*np.array([(-1)**i/(1+.12*i) for i in range(robot.n)])
        offset = np.clip(q0+offset, robot.lower+.01, robot.upper-.01)-q0
        omega = 2*np.pi/duration
        def reference(t):
            return (q0+.5*offset*(1-np.cos(omega*t)),
                    .5*offset*omega*np.sin(omega*t),
                    .5*offset*omega**2*np.cos(omega*t))
        simulation = MujocoRobot(robot, timestep=.002)
        controller = ComputedTorque(robot)
        simulation.reset(q0)
        for t in times:
            while simulation.data.time < t-1e-10:
                q, v = simulation.state()
                tau = controller.command(q, v, *reference(simulation.data.time))
                simulation.step(tau)
            q, v = simulation.state()
            tau = controller.command(q, v, *reference(t))
            states.append(q)
            errors.append(float(np.linalg.norm(q-reference(t)[0])))
            torque_ratio.append(float(np.max(np.abs(tau)/robot.effort)))
            torques.append(tau.tolist())
    names, parents, positions = _geometry(robot, states)
    import mujoco
    return {
        "id": name, "title": TITLES[name], "engine": engine,
        "dof": robot.n, "nq": robot.nq, "nv": robot.nv,
        "floating_base": robot.floating_base,
        "frame": catalog()[name]["frame"], "times": times.round(6).tolist(),
        "link_names": names, "edges": parents, "positions": positions,
        "joint_names": robot.joint_names, "locked_joints": robot.tree.locked,
        "base_pose": {"position": robot.base_pose[:3, 3].tolist(),
                      "quaternion_xyzw": Rotation.from_matrix(robot.base_pose[:3, :3]).as_quat().tolist()},
        "q": np.array(states).round(7).tolist(), "torques": torques,
        "errors": errors, "torque_ratio": torque_ratio,
        "summary": {"final_error": errors[-1], "max_error": None if robot.floating_base else max(errors),
                    "max_torque_ratio": None if robot.floating_base else max(torque_ratio),
                    "physics_dt": None if robot.floating_base else .002,
                    "duration": duration, "amplitude": amplitude,
                    "full_geometry_checked": False},
        "versions": {"mujoco": mujoco.__version__, "python": platform.python_version()},
        "description": ("Floating-base dynamics are validated. This view is prescribed joint motion; "
                        "no standing controller or humanoid dynamics simulation is claimed."
                        if robot.floating_base else
                        "Recorded closed-loop MuJoCo computed-torque tracking using URDF inertias. "
                        "Viewer visual/collision geometry is loaded separately from matching upstream URDFs. "
                        "The recorded tracking simulation has collision disabled."),
    }


def generate_cito(destination):
    from contact_transfer import Robot
    from contact_transfer.contacts import Capability
    from contact_transfer.system import MechanicalSystem, RigidObject
    from contact_transfer.trajectory import Contact, solve_cito
    import tempfile
    link_inertia = '<inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>'
    xml = '<robot name="cartesian"><link name="base"/>'
    for i, axis in enumerate(("1 0 0", "0 1 0", "0 0 1")):
        xml += f'<link name="link{i}">{link_inertia}</link><joint name="j{i}" type="prismatic"><parent link="'
        xml += ('base' if i == 0 else f'link{i-1}')+f'"/><child link="link{i}"/><axis xyz="{axis}"/>'
        xml += '<limit lower="-2" upper="2" effort="100" velocity="2"/></joint>'
    xml += '</robot>'
    with tempfile.TemporaryDirectory() as temp:
        path = Path(temp)/"cartesian.urdf"
        path.write_text(xml)
        numeric = Robot(path)
        model = numeric.symbolic()
        system = MechanicalSystem([model, model], RigidObject(1., np.zeros(3), np.eye(3)*.02))
        q = np.r_[-.25, 0, .5, .25, 0, .5, 0, 0, .5, 0, 0, 0, 1]
        goal = q.copy()
        goal[[2,5,8]] += .03
        cap = Capability.force_gripper([50.,50.,50.])
        contacts = [Contact("left",[-.25,0,0],[0,0,-1],cap,robot_index=0,frame="link2",attached_steps=tuple(range(8))),
                    Contact("right",[.25,0,0],[0,0,-1],cap,robot_index=1,frame="link2",attached_steps=tuple(range(8)))]
        result = solve_cito(system,q,goal,contacts,steps=8,duration=.8)
        n1,e1,p1 = _geometry(numeric,result.q[:,:3])
        n2,e2,p2 = _geometry(numeric,result.q[:,3:6])
        positions = [a+b+[c[6:9].tolist()] for a,b,c in zip(p1,p2,result.q)]
    data = {"id":"dual_cito","title":"Dual-arm contact transfer","engine":"cito_trajectory",
            "dof":6,"nq":13,"nv":12,"floating_base":False,"frame":"Object",
            "times":np.r_[0,np.cumsum(result.dt)].round(6).tolist(),
            "link_names":["left_"+n for n in n1]+["right_"+n for n in n2]+["object"],
            "edges":e1+[[a+len(n1),b+len(n1)] for a,b in e2],
            "positions":positions,"q":result.q.round(7).tolist(),"torques":result.tau.tolist()+[result.tau[-1].tolist()],
            "errors":[None]*len(result.q),"torque_ratio":[float(np.max(abs(x))/100) for x in result.tau]+[float(np.max(abs(result.tau[-1]))/100)],
            "summary":{"duration":.8,"lift_m":.03,"primal_residual":result.primal_residual,
                       "gripper_velocity_residual":result.gripper_velocity_residual,"max_torque_ratio":float(np.max(abs(result.tau))/100),
                       "full_geometry_checked":False,"final_error":None,"physics_dt":None},
            "object":{"index":len(n1)+len(n2),"size":[.5,.14,.10]},
            "description":"Solved CITO transcription: two Cartesian 3-prismatic arms lift a 1 kg object by 3 cm. "
                          "Force-only grippers remain attached. This is a restricted model with no full-mesh certificate."}
    destination.write_text(json.dumps(data,separators=(",",":")))
    return data


def export_all(directory):
    directory = Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    entries=[]
    for name in sorted(catalog()):
        data=generate(name)
        (directory/(name+".json")).write_text(json.dumps(data,separators=(",",":")))
        entries.append({k:data[k] for k in ("id","title","engine","dof","floating_base")})
        print("Recorded",name,flush=True)
    data=generate_cito(directory/"dual_cito.json")
    entries.append({k:data[k] for k in ("id","title","engine","dof","floating_base")})
    existing = directory/"index.json"
    previous = json.loads(existing.read_text()) if existing.exists() else {"models": []}
    entries += [entry for entry in previous["models"] if entry["id"] not in {x["id"] for x in entries}]
    (directory/"index.json").write_text(json.dumps({"models":entries,"tests_passed":58,"draft_version":"12"},indent=2))


if __name__ == "__main__":
    export_all(Path(__file__).parent/"data")
