"""Execute link-aware contraction -> 3-arm realization on a free articulated object."""
import argparse
from pathlib import Path
from contact_transfer.free_articulated import IDS,run_free
from contact_transfer.free_articulated_view import export_free
from contact_transfer.perturbations import WrenchPulse

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--preset',choices=sorted(IDS),default='free_box_lid');p.add_argument('--all',action='store_true');p.add_argument('--output',default='outputs');p.add_argument('--viewer',default='web');p.add_argument('--push',type=float,default=0.,help='world-Y peak N at root, sin² pulse 8–9 s');p.add_argument('--no-contact',action='store_true');a=p.parse_args()
 for id in sorted(IDS) if a.all else [a.preset]:
  pulses=[WrenchPulse('object_urdf_base',8,1,(0,a.push,0))] if a.push else []
  plan,trace,report,flags=run_free(id,Path(a.output)/id,contacts_enabled=not a.no_contact,disturbances=pulses,progress=lambda t,d:print(id,f'{t:g}/{d:g} s',flush=True))
  export_free(plan,trace,report,flags,a.viewer);print(id,'PASS' if report['execution_passed'] else 'FAIL',report['contact_fraction'],flush=True)
if __name__=='__main__':main()
