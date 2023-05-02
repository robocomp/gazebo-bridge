# Entities Control Plugin

This example contains the bare minimum that's necessary to create a Gazebo
system plugin.

## Build

From the root of the `gz-sim` repository, do the following to build the example:

~~~
cd <gazebo-bridge folder>/EntitiesControlPlugin
mkdir build
cd build
cmake ..
make
~~~

This will generate the `EntitiesControl` library under `build`.

## Run

The plugin must be attached to an entity to be loaded.

Before starting Gazebo, we must make sure it can find the plugin by doing:

~~~
cd <gazebo-bridge folder>/EntitiesControlPlugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

Then load a Gazebo world with the plugin attached:

    gz sim -v 3 <world.sdf>
