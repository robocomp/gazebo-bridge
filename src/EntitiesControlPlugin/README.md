# Entities Control Plugin


## Build

Do the following to build the plugin:

~~~
cd <gazebo-bridge folder>/src/EntitiesControlPlugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `EntitiesControl` library under `build`.

## Run


The plugin must be attached inside a <world> tag to be loaded. For example:

~~~ xml
<world name='shapes'>
    <plugin filename="EntitiesControl" name="gz::sim::v7::systems::EntitiesControl">
    </plugin>
</world>
~~~

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~ bash
cd <gazebo-bridge folder>/src/EntitiesControlPlugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load a Gazebo world with the plugin attached:

~~~ bash
gz sim -v 3 <world.sdf>
~~~

## Implemented Services

| Service                | Required Type |                                                                                                                                                                           Info |
|------------------------|:-------------:|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| `get_world_position`   |   StringMsg   |                                                                           Passing the name of a model as a parameter, it generates a topic that publishes its global position. |
| `set_linear_velocity`  |     Pose      | Sets a linear speed to a particular link given its name. Through the field 'name' the name of the link is determined and through 'position' the velocity vector to be applied. |