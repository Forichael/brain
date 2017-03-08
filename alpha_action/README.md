## Alpha\_Action

Defines all the actions that the robot will be capable of.
Currently, it only implements the gripper interface.

### To Test:

Run The Server:

```bash
rosrun alpha_action alpha_gripper_server
```

Run The Client:

```bash
rosrun alpha_action alpha_gripper_client
```

Alternatively, (CLI)

```bash
rostopic pub /alpha_grip_goal alpha_action/GripActionGoal <TAB-COMPLETE>
```

After you tab-complete, just fill in the section "do\_grip" with your desired action
