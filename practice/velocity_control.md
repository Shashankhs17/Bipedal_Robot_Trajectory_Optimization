```python
import pybullet as p

import time
```

    pybullet build time: Oct 11 2021 21:00:24



```python
p.connect(p.GUI)
```




    0




```python
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
```


```python
plane = p.loadURDF('plane.urdf')
```


```python
robot = p.loadURDF('husky/husky.urdf')
```


```python
p.getNumBodies()
```




    2




```python
numJoints = p.getNumJoints(robot)

numJoints
```




    10




```python
for i in range(numJoints):
    print(p.getJointInfo(robot, i))
```

    (0, b'chassis_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.14493), (0.0, 0.0, 0.0, 1.0), -1)
    (1, b'imu_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'imu_link', (0.0, 0.0, 0.0), (0.08748, 0.00085, 0.09053), (0.0, 0.0, 0.0, 1.0), 0)
    (2, b'front_left_wheel', 0, 7, 6, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_left_wheel_link', (0.0, 1.0, 0.0), (0.34348, 0.28625, -0.06665), (0.0, 0.0, 0.0, 1.0), 0)
    (3, b'front_right_wheel', 0, 8, 7, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_right_wheel_link', (0.0, 1.0, 0.0), (0.34348, -0.28454999999999997, -0.06665), (0.0, 0.0, 0.0, 1.0), 0)
    (4, b'rear_left_wheel', 0, 9, 8, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_left_wheel_link', (0.0, 1.0, 0.0), (-0.16852, 0.28625, -0.06665), (0.0, 0.0, 0.0, 1.0), 0)
    (5, b'rear_right_wheel', 0, 10, 9, 1, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_right_wheel_link', (0.0, 1.0, 0.0), (-0.16852, -0.28454999999999997, -0.06665), (0.0, 0.0, 0.0, 1.0), 0)
    (6, b'top_plate', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'top_plate_link', (0.0, 0.0, 0.0), (0.08748, 0.00085, -0.09947), (0.0, 0.0, 0.0, 1.0), 0)
    (7, b'user_rail', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'user_rail_link', (0.0, 0.0, 0.0), (0.35948, 0.00085, 0.14553), (0.0, 0.0, 0.0, 1.0), 0)
    (8, b'front_bumper', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'front_bumper_link', (0.0, 0.0, 0.0), (0.56748, 0.00085, -0.008470000000000005), (0.0, 0.0, 0.0, 1.0), 0)
    (9, b'rear_bumper', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'rear_bumper_link', (0.0, 0.0, 0.0), (-0.39252, 0.00085, -0.008470000000000005), (0.0, 0.0, 0.9999999999991198, -1.3267948966775328e-06), 0)

#### The joints responsible for wheels are 2, 3, 4 & 5

```python
p.setGravity(0.0, 0.0, -9.8)

p.setTimeStep(1.0)

p.setJointMotorControlArray(robot, range(2, 6), p.VELOCITY_CONTROL, targetVelocities = [0.5] * 4)
```


```python
for _ in range(10000):
    p.stepSimulation()
    time.sleep(0.1)
```
