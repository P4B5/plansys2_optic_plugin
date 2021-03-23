# plansys2_optic_plugin 
plugging for Plansys2 planner

# Documentation

To add plansys2 optic plugin to plansys2 

```
git clone https://github.com/P4B5/plansys2_optic_plugin.git
mv /plansys2_optic_plugin/plansys2_optic_plan_solver ros2_planning_system
```
> modify /ros2_planning_system/params/plansys2_params.yaml to add OPTIC

```
planner:
  ros__parameters:
    plan_solver_plugins: ["OPTIC"]
    POPF:
      plugin: "plansys2/POPFPlanSolver"
    TFD:
      plugin: "plansys2::TFDPlanSolver"
    OPTIC:
      plugin: "plansys2/OPTICPlanSolver"

```

> modify lines 59 and 118 of plansys2_optic_plans_solver/src/optic_plan_solver.cpp to your correct path to the optic executable

```
cd ~/your_ws
colcon build --symlink-install
```

> See the examples dirs to execute some of them 



# Other info
## OPTIC Instalation

instalation: https://nms.kcl.ac.uk/planning/software/optic.html

wiki: https://planning.wiki/ref/planners/optic

## Support OPTIC

| Requeriment | Suported |
| ------------- | ------------- |
| :strips | Yes | 
| :typing	| Yes |
| :equality	| Yes | 
| :requeriments	| Yes |
| :universal-preconditions | Yes |
| :numeric-fluents	| Yes | 
| :durative-actions | Yes | 
| :durative-inequalities | Yes | 
| :continuous-effects	| Yes |
|PDDL2.2|
| :timed-initial-literals	| Yes
|PDDL3.0|
:constraints	| Yes
:preferences	| Yes
| PDDL 3.1 | 
| :constraints |	Yes | 
| :preferences | 	Yes | 
