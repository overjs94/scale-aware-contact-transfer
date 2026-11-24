"""Reproduce finite-step grasp mismatch without an optimizer or CasADi."""
import json
from pathlib import Path
import numpy as np
from contact_transfer.panel_fixture import PanelCase
from contact_transfer.paper_validation import grasp_discretization_probe


def main():
    case=PanelCase.load();robot=case.robots[0];q=np.asarray(case.data['arms'][0]['q_initial'])
    velocity=np.array([.10,-.12,.08,.06,-.07,.09,.05]);steps=[.4,.2,.1,.05,.025,.0125]
    rows=grasp_discretization_probe(robot,'tip',q,velocity,steps)
    control=velocity.copy();control[1:]=0
    report=dict(robot='original Panda URDF',method='native Pinocchio FK/J/log6 only; no NLP or CasADi',continuous_grasp_exact_by_construction=True,joint_Euler_step_exact=True,object_SE3_step_exact_by_log=True,simultaneous_post_step_velocity_residual=rows,single_joint_control_residual=grasp_discretization_probe(robot,'tip',q,control,[.4])[0]['max_residual'],interpretation='A prescribed continuous grasp trajectory generally does not exactly satisfy all finite-step closure/integration/instantaneous-velocity constraints. This does not prove the free-trajectory NLP infeasible. The discrepancy shrinks under time refinement.')
    path=Path('outputs/paper_panel/discretization-diagnosis.json');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
