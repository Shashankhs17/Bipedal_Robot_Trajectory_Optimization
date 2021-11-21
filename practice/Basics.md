# PyBullet - Basics

PyBullet is an easy to use Python module for physics simulation for robotics, games, visual effects and machine learning. With PyBullet you can load articulated bodies from URDF, SDF, MJCF and other file formats. PyBullet provides forward dynamics simulation, inverse dynamics computation, forward and inverse kinematics, collision detection and ray intersection queries.

Aside from physics simulation, there are bindings to rendering, with a CPU renderer (TinyRenderer) and OpenGL visualization and support for Virtual Reality headsets such as HTC Vive and Oculus Rift. PyBullet also has functionality to perform collision detection queries (closest points, overlapping pairs, ray intersection test etc) and to add debug rendering (debug lines and text). PyBullet has cross-platform built-in client-server support for shared memory, UDP and TCP networking.


```python
import pybullet as p
```

    pybullet build time: Oct 11 2021 21:00:24


After importing the PyBullet module, the first thing to do is 'connecting' to the physics simulation. PyBullet is designed around a client-server driven API, with a client sending commands and a physics server returning the status. PyBullet has some built-in physics servers: DIRECT and GUI. Both GUI and DIRECT connections will execute the physics simulation and rendering in the same process as PyBullet.

The DIRECT connection sends the commands directly to the physics engine, without using any transport layer and no graphics visualization window, and directly returns the status after executing the command. 

The GUI connection will create a new graphical user interface (GUI) with 3D OpenGL rendering, within the same process space as PyBullet.


```python
physicsClientID = p.connect(p.GUI)
```

#### Connection status and type with server


```python
p.getConnectionInfo(physicsClientID)
```




    {'isConnected': 1, 'connectionMethod': 1}



## Setup

#### To load and use pybullet exmaples


```python
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
```

#### To load urdf 

The other parameters that can be given to loadURDF() are start_position, start_orientation, useFixedBase = 0 if no & 1 if yes, physicsClientID


```python
plane = p.loadURDF('plane.urdf')
```


```python
import os

os.system("git clone https://github.com/ros-industrial/kuka_experimental.git")
```

    Cloning into 'kuka_experimental'...





    0



### Start position and Orientation can be given to loadURDF and the follwoing functions are used to create the data

The PyBullet API uses quaternions to represent orientations. Since quaternions are not very intuitive, there are two APIs to convert between quaternions and Euler angles.

###### getQuaternionFromEuler() and getEulerFromQuaternion()


```python
StartPos = [0,0,1]
StartOrientation = p.getQuaternionFromEuler([0,0,0])
```


```python
robot = p.loadURDF("kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf",StartPos, StartOrientation, useFixedBase = 1)

robot
```

    startThreads creating 1 threads.
    starting thread 0
    started thread 0 
    argc=2
    argv[0] = --unused
    argv[1] = --start_demo_name=Physics Server
    ExampleBrowserThreadFunc started
    X11 functions dynamically loaded using dlopen/dlsym OK!
    X11 functions dynamically loaded using dlopen/dlsym OK!
    Creating context
    Created GL 3.3 context
    Direct GLX rendering context obtained
    Making context current
    GL_VENDOR=Intel
    GL_RENDERER=Mesa Intel(R) UHD Graphics (CML GT2)
    GL_VERSION=4.6 (Core Profile) Mesa 21.0.3
    GL_SHADING_LANGUAGE_VERSION=4.60
    pthread_getconcurrency()=0
    Version = 4.6 (Core Profile) Mesa 21.0.3
    Vendor = Intel
    Renderer = Mesa Intel(R) UHD Graphics (CML GT2)
    b3Printf: Selected demo: Physics Server
    startThreads creating 1 threads.
    starting thread 0
    started thread 0 
    MotionThreadFunc thread started
    ven = Intel
    Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu
    ven = Intel
    Workaround for some crash in the Intel OpenGL driver on Linux/Ubuntu
    b3Printf: b3Warning[examples/Importers/Imp




    1



    ortURDFDemo/BulletUrdfImporter.cpp,126]:
    
    b3Printf: No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
    
    b3Printf: tool0
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
    
    b3Printf: No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frame
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
    
    b3Printf: Link1
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,1159]:
    
    b3Printf: kuka_experimental/kuka_kr210_support/urdf/: cannot extract anything useful from mesh 'kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/base_link.dae'
    
    b3Printf: b3Warning[examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp,558]:
    
    b3Printf: issue extracting mesh from COLLADA/STL file kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/base_link.dae
    
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,1159]:
    
    b3Printf: kuka_experimental/kuka_kr210_support/urdf/: cannot extract anything useful from mesh 'kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_1.dae'
    
    b3Printf: b3Warning[examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp,558]:
    
    b3Printf: issue extracting mesh from COLLADA/STL file kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_1.dae
    
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,1159]:
    
    b3Printf: kuka_experimental/kuka_kr210_support/urdf/: cannot extract anything useful from mesh 'kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_2.dae'
    
    b3Printf: b3Warning[examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp,558]:
    
    b3Printf: issue extracting mesh from COLLADA/STL file kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_2.dae
    
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,1159]:
    
    b3Printf: kuka_experimental/kuka_kr210_support/urdf/: cannot extract anything useful from mesh 'kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_3.dae'
    
    b3Printf: b3Warning[examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp,558]:
    
    b3Printf: issue extracting mesh from COLLADA/STL file kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_3.dae
    
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,1159]:
    
    b3Printf: kuka_experimental/kuka_kr210_support/urdf/: cannot extract anything useful from mesh 'kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_4.dae'
    
    b3Printf: b3Warning[examples/SharedMemory/plugins/tinyRendererPlugin/TinyRendererVisualShapeConverter.cpp,558]:
    
    b3Printf: issue extracting mesh from COLLADA/STL file kuka_experimental/kuka_kr210_support/meshes/kr210l150/visual/link_4.dae
    
    b3Printf: b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImp

#### getBasePositionAndOrientation(ID) gives the [x, y, z] Position and Orientation in Quaternions


```python
position, orientation = p.getBasePositionAndOrientation(robot)
position, orientation
```




    ((-0.027804, 0.00039112, 1.14035), (0.0, 0.0, 0.0, 1.0))



#### To get number of bodies present


```python
p.getNumBodies()
```




    2



#### To get number of joints in specified robot


```python
p.getNumJoints(robot)
```




    8



### getJointInfo() gives various information about specified joint

#### The getJointInfo returns a list of information:

| Data             	| Type   	| Description                                                                                                                                                                                                                 	|
|:------------------	|:--------:	|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| jointIndex       	| int    	| the same joint index as the input parameter                                                                                                                                                                                 	|
| jointName        	| string 	| the name of the joint, as specified in the URDF (or SDF etc) file                                                                                                                                                           	|
| jointType        	| int    	| type of the joint, this also implies the number of position and velocity variables. JOINT_REVOLUTE, JOINT_PRISMATIC, JOINT_SPHERICAL, JOINT_PLANAR, JOINT_FIXED. See the section on Base, Joint and Links for more details. 	|
| qIndex           	| int    	| the first position index in the positional state variables for this body                                                                                                                                                    	|
| uIndex           	| int    	| the first velocity index in the velocity state variables for this body                                                                                                                                                      	|
| flags            	| int    	| reserved                                                                                                                                                                                                                    	|
| jointDamping     	| float  	| the joint damping value, as specified in the URDF file                                                                                                                                                                      	|
| jointFriction    	| float  	| the joint friction value, as specified in the URDF file                                                                                                                                                                     	|
| jointLowerLimit  	| float  	| Positional lower limit for slider and revolute (hinge) joints                                                                                                                                                               	|
| jointUpperLimit  	| float  	| Positional upper limit for slider and revolute joints. Values ignored in case upper limit <lower limit                                                                                                                      	|
| jointMaxForce    	| float  	| Maximum force specified in URDF (possibly other file formats) Note that this value is not automatically used. You can use maxForce in 'setJointMotorControl2'                                                               	|
| jointMaxVelocity 	| float  	| Maximum velocity specified in URDF. Note that the maximum velocity is not used in actual motor control commands at the moment                                                                                               	|
| linkName         	| string 	| the name of the link, as specified in the URDF (or SDF etc.) file                                                                                                                                                           	|
| jointAxis        	| vec3   	| joint axis in local frame (ignored for JOINT_FIXED)                                                                                                                                                                         	|
| parentFramePos   	| vec3   	| joint position in parent frame                                                                                                                                                                                              	|
| parentFrameOrn   	| vec3   	| joint orientation in parent frame                                                                                                                                                                                           	|
| parentIndex      	| int    	| parent link index, -1 for base                                                                                                                                                                                              	|



```python
joint_index = 6

joint_info = p.getJointInfo(robot, joint_index)

joint_info
```




    (6,
     b'link_6-tool0',
     4,
     -1,
     -1,
     0,
     0.0,
     0.0,
     0.0,
     -1.0,
     0.0,
     0.0,
     b'tool0',
     (0.0, 0.0, 0.0),
     (0.055456, 1.5237e-05, -0.00039408),
     (0.0, 0.0, 0.0, 1.0),
     5)



## Simulation


#### By default, there is no gravitational force enabled. ​setGravity() ​is used to set gravity for all objects
```python
p.setGravity(0, 0, -9.81)
```

```python
# To reset the simulation
# p.resetSimulation()         -> This command resets the simulation and removes the spawned urdf 

# To reset position and orientation of robot
p.resetBasePositionAndOrientation(robot, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
```

By default, the physics server will not step the simulation, unless explicitly the 'stepSimulation' command is sent. This way one can maintain control determinism of the simulation. It is possible to run the simulation in real-time by letting the physics server automatically step the simulation according to its real-time-clock (RTC) using the setRealTimeSimulation command. If real-time simulation is enabled, there won't be need to call 'stepSimulation'. 

The **setTimeStep()** sets the interval between each steps


```python
p.setTimeStep(0.5)

p.setRealTimeSimulation(0)
```

We can control a robot by setting a desired control mode for one or more joint motors. During the stepSimulation the physics engine will simulate the motors to reach the given target value that can be reached within the maximum motor forces and other constraints. Each revolute joint and prismatic joint is motorized by default. There are 3 different motor control modes: position control, velocity control and torque control.

The motor can be effectively disabled by using a force of 0. You need to disable motor in order to use direct torque control. 


```python
import time
```


```python
p.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL, targetPositions = [1.0] * 6)
```


```python
for _ in range(10):
    p.stepSimulation()
    time.sleep(0.5)
```

##### To disconnect from the server

```python
p.disconnect()
```