"""Run an actual native position actuator; no hardware connection."""
from contact_transfer.rb_position import run_rb5_position_servo
if __name__=='__main__':
    report,_=run_rb5_position_servo();print(report)
    if not report['execution_passed']:raise SystemExit(1)
