**Controller input**

Input vector: `[av_speed, leader_speed, headway]`

where

- `av_speed` is the speed of the AV (in m/s)
- `leader_speed` is the speed of the AV's leader (in m/s)
- `headway` is the bumper-to-bumper gap between the AV and its leader (in m)

Note that **no normalization should be applied** to the input vectors.

**Controller output**

Output vector: `[accel]`

where

- `accel` is an acceleration (in m/s^2)

**Controller settings**

The controller is a Time-Headway Follower-Stopper.

The desired speed should be set as `v_des = av_speed + T * accel` at each time step.

`T` (integration time step in seconds) is a design parameter. Set `T = 0.6`.

If no leader is detected, set `leader_speed = 30` and `headway = 150` (or whatever is already implemented).

**I/O examples**

```
inputs = [
    [5, 5, 20],
    [5, 20, 30],
    [30, 10, 50],
    [30, 30, 110],
    [30, 30, 120],
]

outputs = [
    0.0078,  
    1.4998, 
    -1.1934,  
    0.4371,  
    0.4874,
]
```
