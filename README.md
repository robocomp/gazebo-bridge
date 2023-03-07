# Gazebo2Robocomp

<p> This Robocomp component is responsible for communicating Robocomp with Gazebo
simulator. You can use it in any Gazebo simulation in order to collect data through 
other Robocomp components or sending topic and commands to the simulation.
</p>

## Requirements


<p> First of all, in order to execute the component you need to install 
<a href="https://github.com/robocomp/robocomp">Robocomp</a>.
</p>

<p> Next, you need to install the <a href="https://gazebosim.org/docs/garden/install_ubuntu">Gazebo</a> simulator. (Gazebo Garden is the latest version tested.)
</p>

<p> Lastly, you need to install the <strong>gz-transport12</strong> library.
</p>

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install libgz-transport12
sudo apt-get install libgz-transport12-dev
```

<p> Now you are ready to run the component. Remember that for the component 
to be able to collect data, the simulator and the simulation must be started.
</p>



## Running

<p> First of all, you need to generate the component files. Go to the file where
the component is located and execute </p>

```bash
cmake .
make
```

<p> If the compilation is successful you can now run the component </p>

```bash
./bin/Gazebo2Robocomp etc/config 
```

<p> Notice that we enter via parameter a config file. This file can be edited or duplicated 
for been used with your own parameter values. </p>


## Config Parameters

<p> Actually, a config file for this component looks like this: </p>

    CommonBehavior.Endpoints=tcp -p 10217
    
    # Endpoints for implements interfaces
    CameraRGBDSimple.Endpoints=tcp -p 10096
    Laser.Endpoints=tcp -p 10003
    IMU.Endpoints=tcp -p 10005
    JointMotorSimple.Endpoints=tcp -p 10006
    OmniRobot.Endpoints=tcp -p 10004
    
    
    # Endpoints for subscriptions interfaces
    JoystickAdapterTopic.Endpoints=tcp -p 11025
    
    
    # This property is used by the clients to connect to IceStorm.
    TopicManager.Proxy=IceStorm/TopicManager:default -p 9999
    
    InnerModelPath = innermodel.xml
    
    # Custom parameters
    odometry_target_name = simple_robot
    
    Ice.Warn.Connections=0
    Ice.Trace.Network=0
    Ice.Trace.Protocol=0
    Ice.MessageSizeMax=20004800

<p> This config file is basically a normal Robocomp config file, the only distinction 
comes through the custom parameters.
</p>

    # Custom parameters
    odometry_target_name = simple_robot

<p> This parameter determines the model in the Gazebo simulation on which 
the component will record the odometry data.
If your object on which you want to record it has the name 
of "robot_model" or "car1" or any other name you need to specify it
in the odometry_target_name parameter.
</p>

<p> The recommendation is that when you start working with the component 
you create your own config file where you can make the changes you need.
</p>

```bash
cp etc/config etc/config_myconfig
````

## Supported topics and Robocomp components

<p> Gazebo2Robocomp communicates Robocomp components with Gazebo through a 
system of topics using gz-transport, the topics linked to each supported 
component are the following:
</p>

| Robocomp Component | Gazebo sensor or plugin |         Topic |
|--------------------|:-----------------------:|--------------:|
| CameraRGBDSimple   |         camera          |       /camera |
| CameraRGBDSimple   |      depth_camera       | /depth_camera |
| Laser              |        gpu_lidar        |        /lidar |
| IMU                |           imu           |          /imu |
| JointMotorSimple   |        DiffDrive         |      /cmd_vel |
| JoystickAdapter    |        DiffDrive         |      /cmd_vel |


## Example Usage

<p>
For example, if you want to use Gazebo2Robocomp to move a robot using a Joystick,
you can download this <a href="https://github.com/SergioTrac/GazeboWorld_InvertedPendulum">Inverted Pendulum</a> Gazebo world and follow the
the following:
</p>

### Check the .sdf file

<p>
Check that in the .sdf file of the simulation there is a sensor or 
plugin among those supported and that it is sending the information to 
one of the topics supported by the component.
</p>

<p>
This differential drive plugin would be valid, for example.
</p>

```xml
<!-- Differential drive plugin -->
<plugin
        filename="libignition-gazebo-diff-drive-system.so"
        name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_radius>0.07</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <!-- Subscription to a valid Gazebo2Robocomp topic -->
    <topic>cmd_vel</topic>
</plugin>
```

### Start the simulation and Robocomp components 

<p> Now you can start the downloaded world as a normal Gazebo project
and start the simulation.
</p>

<p>
Then, run the Robocomp components involved, so far: <br>
<strong>Gazebo2Robocomp</strong> for communication between Robocomp and Gazebo <br>
<strong>JoystickAdapter</strong> for sending inputs from the Joystick. You can find this component among 
the <a href="https://github.com/robocomp/robocomp-robolab"> standard Robocomp components</a>.
</p>

<p>
You should now be able to move your robot using the Joystick. Have fun!
</p>


