# PlanSys2 StarWars Example

## Description



## How to run

In terminal 1:

```
ros2 launch plansys2_starwars_example plansys2_starwars_example_launch.py
```

In terminal 2:

```
ros2 run plansys2_terminal plansys2_terminal        # enters in PlanSys2 Terminal
set instance r2d2 robot
set instance c3p0 robot
set instance Naboo planet
set instance Tatooine planet
set instance Kessel planet
set instance Alderaan planet
set instance milleniumFalcon spaceship
set instance Xwing spaceship

get problem instances                               # Checks instances

  set predicate (robotAt r2d2 Naboo)
  set predicate (robotAt c3pO Tatooine)
  set predicate (spaceshipAt milleniumFalcon Alderaan)
  set predicate (spaceshipAt Xwing Kessel)

get problem predicates                                # Checks predicates

set goal (and(robotAt r2d2 Tatooine)(robotAt c3pO Naboo))                # Sets the goal

get plan                                              # Creates plan and shows it
run                                                   # Creates plan and runs it
```
