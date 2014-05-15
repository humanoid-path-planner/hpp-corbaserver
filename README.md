<<<<<<< HEAD
irreducible-configuration-space
===============================

Implementation of the Irreducible configuration space algorithm using HPP

  - Input: Mechanical System, represented by an URDF file
  - Output: Q, a subset of the configuration space of the input system, containing the irreducible set of configurations
  
Algorithmic Structure:

  - Sampling of the free configuration space
  - Each sample is checked for irreducibility
  - Final set approximates the irreducible configuration space

Dependencies:

  - ros-hydro
  - humanoid-path-planner
  - scipy

Start:

  In seperate terminals do
```
  roslaunch hpp_ros hpp_ros.launch
```
```
  ${DEVEL_SRC}/install/bin/hpp-wholebody-step-server
```
```
  cd irreducible-configuration-space/scripts
  ipython
  >>> execfile('test_robot_movement.py')
```
=======
Corba server for Humanoid Path Planner applications.
>>>>>>> 27f1f361e1fbebeda2043f4c03cc24588e9d08f7
