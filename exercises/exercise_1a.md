# Exercise 1a

Experiment with the GUI provided in `noether`.
Your efforts should utimately produce output tool paths that look something like the images below:

![Exercise 1a Image 1](exercise_1a_1.png)

![Exercise 1a Image 2](exercise_1a_2.png)

## Running the GUI

First launch the `noether_gui` app.
This can be done using the ROS2 CLI utilities:

```
source <path>/<to>/<workspace>/install/setup.bash
ros2 run noether_gui noether_gui_app
```

or by manually running the GUI executable:

```
cd <path>/<to>/<workspace>
source install/setup.bash
./install/noether_gui/bin/noether_gui_app
```

### Java Library Loading Issues

On Ubuntu 22.04+, a few Java libraries cannot be found for dynamic loading by default.
Add the following paths to the `LD_LIBRARY_PATH` environment variable to avoid this issue.
Consider adding this change to your `.bashrc` file so you don't have to set it every time.

#### Ubuntu 22.04

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib:/usr/lib/jvm/java-11-openjdk-amd64/lib/server
```

#### Ubuntu 24.04

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-21-openjdk-amd64/lib:/usr/lib/jvm/java-21-openjdk-amd64/lib/server
```

## Tasks

- Load [the `multi_component.ply` mesh](../meshes/multi_component_mesh.ply)
- Create a raster tool path over the whole mesh
- Create a raster tool path over each individual component of the mesh
- Create a raster tool path over each individual component of the mesh, where the tool paths are generated on a plane rather than the mesh surface
- Modify the output tool path such that the end of one raster is adjacent to the start of the next raster (i.e., the raster has a snake-style shape)
- Modify the output tool path such that the x-axis of the waypoints flips from one stroke of the raster to the next
- Add an approach and departure point above the start/end of each raster
- Visualize the tool path lines and modified mesh

## Solution

The solution tool path planner configuration file can be found [here](exercise_1a_solution.yaml).
Load this file into the GUI to see the combination of mesh modifiers, tool path planner, and tool path modifiers used to accomplish the tasks listed above.
