# Exercise 0

Use the following steps to setup a workspace and compile this repository:

1. Create a `colcon` workspace
    ```commandLine
    mkdir -p noether_ws/src
    ```

1. Clone this repository into the `src` directory of the workspace
    ```commandLine
    cd noether_ws/src
    git clone https://github.com/marip8/noether_roscon_2024.git
    cd ..
    ```

1. Install the dependencies
    ```commandLine
    vcs import src < src/noether_roscon_2024/dependencies.repos
    source /opt/ros/<distro>/setup.bash
    rosdep install --from-paths src -iry --skip-keys libvtk
    ```

1. Build
    ```
    colcon build
    ```
