# Scheduler
Notes for how we plan to implement the scheduler for the OT2_workcell.

### Implementation 
* We need to worry about **arm transfers**
* We also need to worry about **dependencies**
* We need to worry about **input** into the system and **output**

Let the user specify "blocks", which are combinations of protocols/transfers that need to happen in order of each other (linked together kind of). These can then be assigned to be completed by the next OT2 available. These "blocks" would be submitted in workflow files (JSON). **All "blocks" / dependencies** should be required in the workflow file. That way we can easily link the dependencies together and run them, it would throw an error if a dependency is unknown for the workflow file. 

### Issues to Consider
* Deadlocks (request arm transfer, but the other OT2/"block" doesn't request an arm transfer at all, then we are stuck in waiting). This is also more of an **arm transfer issue**. 
* Multiple dependendencies, what if someting requires 3, 4, etc. other dependencies, we need to somehow make sure those dependencies are run before our program 
* We are assuming the user will input blocks correctly. The user might not organize the blocks correctly. (This is a minor issue, as it is not currently our job to solve this problem). 
* Efficiency issue, what happens if we request a transfer that will take it a while or wait for a dependency to complete that might take a while, we would be **wasting a whole OT2** for a long period of time. (We need to consider **preempting** "blocks", but this is a later optimization).
* Depending on the approach we also need to be careful of **starvation**, where a "block" doesn't get to run for a indefinite amount of time (theoritically) since other jobs keep running before it.

### Possible Implementation Ideas 
1. **Batch scheduler (FCFS)**  
This schedler would work on a first come first serve basis, each "block" of actions will be held in a queue and allocate to the next available robot and then removed from the queue. This is the **simplest and easiest** to implement. For a first scheduler this is probably better and simplier for a prototype. 

2. Shortest job first scheduler 
3. **Priority scheduler**  
This will allow the user to specify a priority for an action, and will follow the batch scheduler but instead of first come first serve we assign to the highest priority.
4. Multiple queue scheduler 
5. Shortest time next scheduler
6. **Guarenteed scheduling** 
7. Lottery scheduler 



### Goals 
1. **Robot utilization**  
We need to make sure that at any point in time we are using utilizing as much robots as possible. This ensures that we have high parallelization and high efficiency as we don't want to waste resources. 
2. Fairness 
3. **Throughput** (jobs completed per \<unit of time\>)  
We also need to have high completion rate of jobs, the goal of this system is to improve the speed of wet lab bio experiments and experiments in general. 
4. Response time 
5. Meeting deadlines 
6. Predicatability 
7. **Separation of policy and mechanism**  
Ideally we will implement the mechanism (how things are schedule) but we will let the people using the system specify the **policy** used or in other words the order in which items are schedule (like specifications but not the internal how) 
