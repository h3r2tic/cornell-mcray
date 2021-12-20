@echo off
set BAKE=..\kajiya\target\release\bake

%BAKE% --scene "assets/car/car.glb" --scale 1.0 -o car
%BAKE% --scene "assets/car/wheel_left.glb" --scale 1.0 -o wheel_left
%BAKE% --scene "assets/car/wheel_right.glb" --scale 1.0 -o wheel_right
%BAKE% --scene "assets/track/track.gltf" --scale 1.0 -o track
%BAKE% --scene "assets/track/occluder.gltf" --scale 1.0 -o occluder
%BAKE% --scene "assets/track/lights.gltf" --scale 1.0 -o lights
%BAKE% --scene "assets/track/cube.glb" --scale 1.0 -o cube
