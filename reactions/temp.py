import stretch_body.robot

r = stretch_body.robot.Robot()
r.startup()
assert(r.is_calibrated()) # the robot must be homed

r.lift.motor.enable_pos_traj()

r.lift.move_to(1)
r.push_command()