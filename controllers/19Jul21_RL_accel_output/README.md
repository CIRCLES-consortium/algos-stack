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
If several controllers can be tested, we can try values 0.05 and also values around 0.6 like 0.4 and 0.8.

**I/O examples**

```
>>> model.predict([[5, 5, 20]])
[[0.0077]]

>>> model.predict([[30, 10, 50]])
[[-2.1154]]

>>> model.predict([[5, 20, 30]])
[[1.5000]]
```
