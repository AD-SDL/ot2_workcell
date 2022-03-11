# Deadlock Detection / Prevention 

## Arm 
* Scheduler can refuse to schedule blocks that don't have 2 linked transfers (so that one arm won't deadlock) 
* 

## general 
* Heartbeat? Each node sends out a heartbeat picked up by the master, if it doesn't recieve x amount of heartbeats it marks the nodes are ERRORED or different state 