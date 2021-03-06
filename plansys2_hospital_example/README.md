# PlanSys2 Hospital Example

## Description
This is a simple example that shows the basic operation of PlanSys2 using the Hospital exercise.

## How to run

In terminal 1:

```
ros2 launch plansys2_hospital_example plansys2_hospital_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal

set instance room1 room
set instance room2 room
set instance room3 room
set instance room4 room
set instance room5 room
set instance room6 room
set instance room7 room
set instance room8 room
set instance room9 room
set instance corridor1 corridor
set instance corridor2 corridor
set instance corridor3 corridor
set instance zone1 zone
set instance zone2 zone
set instance zone3 zone
set instance zone4 zone
set instance zone5 zone
set instance door1 door
set instance door2 door
set instance door3 door
set instance door4 door
set instance door5 door
set instance door6 door
set instance door7 door
set instance elevator elevator
set instance object1 object
set instance robot robot

get problem instances                               # Checks instances

set predicate (placesTogether room3 room1)
set predicate (placesTogether room1 room3)
set predicate (placesTogether room1 corridor1)
set predicate (placesTogether corridor1 room1)
set predicate (placesTogether corridor1 room2)
set predicate (placesTogether room2 corridor1)
set predicate (placesTogether room2 room4)
set predicate (placesTogether room4 room2)
set predicate (placesTogether zone1 room1)
set predicate (placesTogether zone2 room1)
set predicate (placesTogether zone3 room2)
set predicate (placesTogether zone4 room2)
set predicate (placesTogether zone5 room2)
set predicate (placesTogether room1 zone1)
set predicate (placesTogether room1 zone2)
set predicate (placesTogether room2 zone3)
set predicate (placesTogether room2 zone4)
set predicate (placesTogether room2 zone5)
set predicate (placesFree corridor1 room2)
set predicate (placesFree room2 corridor1)
set predicate (placesFree room2 room4)
set predicate (placesFree room4 room2)
set predicate (placesFree room1 zone1)
set predicate (placesFree room1 zone2)
set predicate (placesFree room2 zone3)
set predicate (placesFree room2 zone4)
set predicate (placesFree room2 zone5)
set predicate (placesFree zone1 room1)
set predicate (placesFree zone2 room1)
set predicate (placesFree zone3 room2)
set predicate (placesFree zone4 room2)
set predicate (placesFree zone5 room2)
set predicate (placesDoor room3 door2 room1)
set predicate (placesDoor room1 door2 room3)
set predicate (placesDoor room1 door1 corridor1)
set predicate (placesDoor corridor1 door1 room1)
set predicate (doorAt door1 corridor1)
set predicate (doorAt door1 room1)
set predicate (doorAt door2 room1)
set predicate (doorAt door2 room3)
set predicate (doorClosed door1)
set predicate (doorClosed door2)
set predicate (placesTogether corridor2 room6)
set predicate (placesTogether room6 corridor2)
set predicate (placesTogether corridor2 room5)
set predicate (placesTogether room5 corridor2)
set predicate (placesTogether corridor2 corridor3)
set predicate (placesTogether corridor3 corridor2)
set predicate (placesTogether corridor3 room7)
set predicate (placesTogether room7 corridor3)
set predicate (placesTogether corridor3 room9)
set predicate (placesTogether room9 corridor3)
set predicate (placesTogether corridor3 room8)
set predicate (placesTogether room8 corridor3)
set predicate (placesFree corridor2 corridor3)
set predicate (placesFree corridor3 corridor2)
set predicate (placesDoor corridor2 door3 room6)
set predicate (placesDoor room6 door3 corridor2)
set predicate (placesDoor corridor2 door4 room5)
set predicate (placesDoor room5 door4 corridor2)
set predicate (placesDoor corridor3 door5 room7)
set predicate (placesDoor room7 door5 corridor3)
set predicate (placesDoor corridor3 door6 room9)
set predicate (placesDoor room9 door6 corridor3)
set predicate (placesDoor corridor3 door7 room8)
set predicate (placesDoor room8 door7 corridor3)
set predicate (doorAt door3 corridor2)
set predicate (doorAt door3 room6)
set predicate (doorAt door4 corridor2)
set predicate (doorAt door4 room5)
set predicate (doorAt door5 corridor3)
set predicate (doorAt door5 room7)
set predicate (doorAt door6 corridor3)
set predicate (doorAt door6 room9)
set predicate (doorAt door7 corridor3)
set predicate (doorAt door7 room8)
set predicate (doorClosed door3)
set predicate (doorClosed door4)
set predicate (doorClosed door5)
set predicate (doorClosed door6)
set predicate (doorClosed door7)
set predicate (elevatorAt elevator corridor1)
set predicate (elevatorAt elevator corridor2)
set predicate (robotAt robot room1)
set predicate (robotIdle robot)
set predicate (objectAt object1 zone1)

get problem predicates                                # Checks predicates

set goal (and(robotAt robot room8))                # Sets the goal

# Other posible goals
set goal (and(robotAt robot zone5)(objectAt object1 room7))
set goal (and(robotAt robot room9)(objectAt object1 zone3)(doorClosed door1)(doorClosed door6)(doorOpened door5))    


get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
