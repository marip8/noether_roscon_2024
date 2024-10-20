#! /bin/bash
source /opt/noether_roscon_2024/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib:/usr/lib/jvm/java-11-openjdk-amd64/lib/server
./opt/noether_roscon_2024/install/noether_gui/bin/noether_gui_app
