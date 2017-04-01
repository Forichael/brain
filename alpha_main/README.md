## Alpha\_Main

All of the project-wide code, involving an integration of multiple processing units, belong to this repository. Currently, it mostly serves as a meta-package that includes the launch files, as well as a state-machine node.

### Running the State Machine

```bash
roslaunch alpha_main state_machine.launch
```

#### Cautions

Sometimes the state machine doesn't exit by just triggering SIGINT(Ctrl+C).

In this event, follow the guideline below:

1. Find Whether or not there's still a node running:

```bash
pgrep states -fl
```

2. If the result is something like:

```bash
9999 python
```

3. Then kill the process as follows:

```bash
kill -9 9999
```

And don't forget to change 9999 to the PID found in step 2.
