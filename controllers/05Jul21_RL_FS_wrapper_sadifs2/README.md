**Controller input**

Input vector: `[av_speed / 40.0, leader_speed / 40.0, headway / 100.0, 0.0, 0.0, 0.0]`

where

- `av_speed` is the speed of the AV (in m/s)
- `leader_speed` is the speed of the AV's leader (in m/s)
- `headway` is the bumper-to-bumper gap between the AV and its leader (in m)

The last 3 states are padding.

**Controller output**

Output vector: `[v_des_delta]`

The controller is a Time-Headway Follower-Stopper.

At each time step, its desired velocity should be set to `v_des = leader_speed + v_des_delta`.

**I/O examples**

```
>>> model.predict([0.14855019, 0.1362198, 0.13667326 0.0, 0.0, 0.0])
[-0.01187114]

>>> model.predict([0.52693541, 0.5538559, 0.25393719, 0.0, 0.0, 0.0])
[0.03311742]

>>> model.predict([0.63994685, 0.64963934, 0.29815035, 0.0, 0.0, 0.0])
[0.01801384]
```
