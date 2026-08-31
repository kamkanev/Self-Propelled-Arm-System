## Design and enviroment etc.

System state machine definition, specification of capabilites.

### Hardware description:  
Base: pure actuator no sensor, controlled with empirical parameters.  
Camera: main(only) sensor, 300*240 pixel   
Claw: low precision electric current.   
Any documentations from the retailer's website.

### Optional math:   
Localization with AprilTag upon throwing the can to the bin using it's corners(or discuss another approach with two AprilTag).    
Textbook inertial Navigation for searching the bin.   
Avoidance path computation.   

## Minimal verification attempt:  
### Specs & invariants(semi-formal language):
1. What can our system do, ideally? A toy controller works similar to Tesla?
2. what can our system do, realistically using our setting, like a 320*200 camera that sees nothing clearly futher than 3 meters, some limited computation power, limited neural models, openCV's classic AprilTag algorithm etc..
3. Invariant 1: If the tank is close(distance <= k cm) to the AprilTag, the car can always obtain the exact shape(four vertices of the square) of the ApriTag.

4.  spec1, spec2, spec3...

### LTL formalization & prove:
....blablabla ABC->DEF so..

## Implementation report
Actual Observation/implementation:  
System liveness, faliure patterns...

## Future works: 
improve this design, buy/integrate that to make the system pass stricter verification.