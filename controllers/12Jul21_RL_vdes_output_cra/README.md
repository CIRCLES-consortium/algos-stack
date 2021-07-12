**Controller input**

Input vector: `[av_speed, leader_speed, headway]`

where

- `av_speed` is the speed of the AV (in m/s)
- `leader_speed` is the speed of the AV's leader (in m/s)
- `headway` is the bumper-to-bumper gap between the AV and its leader (in m)

Note that **no normalization should be applied** to the input vectors.

**Controller output**

Output vector: `[v_des]`

The controller is a Time-Headway Follower-Stopper.

At each time step, its desired velocity should be set to the output of the controller.

**I/O examples**

This controller works for a constant operation frequency of 20Hz, ie. a timestep of 0.05s. 

```
>>> model.predict([[5, 5, 20]])
[[5.0004]]

>>> model.predict([[5, 20, 30]])
[[5.0750]]

>>> model.predict([[30, 10, 50]])
[[29.8942]]
```