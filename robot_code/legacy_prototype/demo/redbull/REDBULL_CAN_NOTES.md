# 250 ml Red Bull Can Working Notes

This file records temporary engineering assumptions for the Red Bull pickup demo. It is not a product specification sheet. The goal is to keep the robot-control code readable and make later calibration decisions explicit.

## Current Assumption

This demo now targets the small Red Bull can commonly found in European supermarkets: **250 ml / 8.4 fl oz**. It is treated as a slim aluminium energy-drink can.

Sources:

- Red Bull Q&A, common sizes: 250 ml, 355 ml, 473 ml, sometimes 591 ml: https://www.redbull.com/us-en/energydrink/questions/what-sizes-does-red-bull-come-in
- Red Bull sustainability/lifecycle page: 250 ml aluminium can is listed as 11 g: https://www.redbull.com/int-en/energydrink/red-bull-can-lifecycle
- Crown 250 ml slim can: diameter 53 mm, height 133 mm: https://www.crowncork.com/beverage-packaging/products/beverage-cans/83oz-250ml-slim
- Orora 250 ml slim can: diameter 53 mm, height 133 mm: https://www.ororabeverage.com/products/cans/slim-can-250ml

## Working Physical Parameters

Preferred demo assumption: 250 ml slim Red Bull-style can.

| Parameter | Working value | Notes |
| --- | ---: | --- |
| Nominal volume | 250 ml | Common small Red Bull size |
| Body diameter | 53 mm | Crown/Orora 250 ml slim can value |
| Height | 133 mm | Crown/Orora 250 ml slim can value |
| Radius | 26.5 mm | Derived from 53 mm diameter |
| Empty can mass | about 11 g | Red Bull sustainability page |
| Liquid mass estimate | 250-260 g | Approx. 1.00-1.04 g/ml beverage density assumption |
| Full can mass estimate | 261-271 g | Empty can + liquid estimate |

## Robot-Control Implications

- The can is assumed upright on the floor and stationary.
- Detection can initially use a bottle/can detector or a red/blue object detector; exact Red Bull recognition is future work.
- Approach should stop before the can is under the camera, leaving enough clearance for the arm.
- Because the demo does not yet estimate can weight or grip force, the gripper should use a conservative "firm close + settle + re-close" sequence.
- A full 250 ml can may be roughly 0.26-0.27 kg, so lifting still requires more caution than an empty can.
- This demo should first be tested on an empty can, then a partially filled can, before trying a full can.

## Current Simplified Demo Flow

1. Rotate in place and scan frames.
2. Detect a Red Bull-like can at an assumed distance, e.g. 5 m.
3. Align until the can is near the image center.
4. Drive forward in small segments.
5. Re-estimate distance after each segment.
6. Stop at a coarse grasp staging distance.
7. Send relative can distance/offset to the arm routine.
8. Keep chassis stopped.
9. Execute simple arm poses and a firm gripper close.
10. Log servo target positions to an external log file.
11. Lift the can.
12. Stop the process for inspection.
