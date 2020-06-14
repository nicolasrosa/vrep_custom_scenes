# Trabalho Final



1. Open V-REP (CoppeliaSim)

   ```shell
   $ cd <Path>/vrep/
   $ ./coppeliaSim.sh
   ```

2. Select modified scene:

   1. mapSensor added (World Top View)
   2. Main Script File modified to support RemoteApi (Python)
   3. Pioneer_p3dx
      1. GPS (Noisy Position)
      2. Compass (Noisy/Discrete Orientation)

3. Start Simulation (Play Button)

4. Run the python script

   ```shell
   $ python3 remoteApi_pioneer_trab_final_threaded.py
   ```

5. On the "D* Lite Path Planning"

   1. `space` key: Run planner step-by-step.
   2. `tab` key: Run planner until find the complete path to goal. Navigation by `follow_waypoints` starts immediately after thats.
   3. The navigation runs until the waypoints list is empty last waypoint is the target position (goal).

