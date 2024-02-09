#bin/bash

xacro robot.urdf.xacro > model.urdf
gz sdf -p model.urdf | sed 's/version='\''1.10'\''/version='\''1.9'\''/' > robot.urdf.sdf
rm model.urdf