# SSC0714/SSC5888 - Trabalho Final

## Run Commands

1. Open V-REP (CoppeliaSim)

   ```shell
   $ cd /home/<user>/<path_to_vrep>/
   $ ./coppeliaSim.sh
   ```

2. Pick a modified scene inside `vrep_custom_scenes/nicolasrosa/trab_final` folder:

   1. `TP-CP-Cenario01-2020_nick_python.ttt`
   2. `TP-CP-Cenario01-2020_nick_python.ttt`
   3. `TP-CP-Cenario01-2020_nick_python.ttt`
   4. `TP-CP-Cenario01-2020_nick_python.ttt`

3. Start V-REP/CoppeliaSim simulation (Play Button)

4. Run the python script

   ```shell
   $ python3 remoteApi_pioneer_trab_final_threaded.py
   ```

5. On the "D* Lite Path Planning" window

   1. `space` key: Run planner step-by-step.
   2. `tab` key: Planner runs until find the complete path to goal. Navigation by `follow_waypoints` starts immediately after that.
   3. The navigation runs until the way-points list is `empty`. Last way-point corresponds to the target position (goal).



## Scenes Modifications

1. `mapSensor` added (World Top View);
2. Scene Main Script File was modified to support `RemoteApi (Python)`;



## Robot Platform Modifications

1. `Pioneer_p3dx`:
   1. `GPS` (Noisy Position)
   2. `Compass` (Noisy/Discrete Orientation)
   3. `Graph` (Trajectory)

