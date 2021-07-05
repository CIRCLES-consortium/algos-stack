**Controller input**

Input vector: `[av_speed / 40.0, leader_speed / 40.0, headway / 100.0]`

where

- `av_speed` is the speed of the AV (in m/s)
- `leader_speed` is the speed of the AV's leader (in m/s)
- `headway` is the bumper-to-bumper gap between the AV and its leader (in m)

**Controller output**

Output vector: `[v_des_delta]`

The controller is a Time-Headway Follower-Stopper.
At each time step, its desired velocity should be set to `v_des = leader_speed + v_des_delta`.