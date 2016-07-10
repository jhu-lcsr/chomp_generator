CHOMP Generator
===============

This package uses the CHOMP motion planner to generate trajectories in an
environment.


Requirements
------------

- Subscribe to moveit planning scene
  - get robot state
  - get obstacles
- Generate (n) trajectories from one EE pose to another EE pose
  - Generate trajectory i
  - Add it to the list of avoidance trajectories
- 


Needs:

```
PlanningSceneMonitor < robot_description
PlanningPipeline pieline < PlanningSceneMonitor

PlannerManaer < PlanningPipeline

for 1 to n:
  pipeline.generatePlan()
end
```
