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

`T` (integration time step in seconds) is a design parameter. Try setting `T = 0.6` first. 
If several controllers can be tested, can try `T = 1.0` as well. 

If no leader is detected, set `leader_speed = 30` and `headway = 150`.

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
    0.00767505,  
    1.0, 
    -2.115352,  
    0.5054928,  
    1.0,
]
```
