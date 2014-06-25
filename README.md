matlab-rigid-body-viz
=====================

3D Rigid Body Visualization

This MATLAB library is meant to render 3D objects by building off some basic
shape primitives.  The encompassing design is to 'create' the rigid bodies 
at the beginning of the program and then 'update' them using rigid body
transformations (i.e. rotation + translation).  

This is also built into a robot-friendly framework, where serial-chain 
robots can be generated and then updated using forward kinematics.
