## RHR:
* X towards front of robot
* Thumb towards front of robot (+X)
* CW+ (I.e. +ve angle more you get towards rear)

## Current
* 0 for lower points forward parallel to ground
* 0 for upper points forward parallel to ground when lower is at 90
  * This is wrong and isn't specified in the Javadoc for upper encoder offsets...

## Comparison
Convention | Lower | Upper | Zero
--- | --- | --- | ---
Current | CW +ve | CW +ve | parallel to ground when lower at 90
RHR | CW +ve | CW +ve | N/A
Gravity | CCW +ve | CCW +ve |

## Positive Directions
| Mechanism | Motor | Encoder |
| --- | --- | --- |
| Lower | CCW -> CW since is mounted on opposite end of shaft | CW | 
| Upper | CCW -> CW since is mounted on opposite end of shaft | CW |