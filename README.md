# Rib Cage

This is currently a collection of physics and animation projects / experiments built on top of my rendering engine [Althea](https://github.com/nithinp7/Althea). Below are the projects so far: 

### Cloth Self Collision

This is primarily a study on continuous collision detection and constraint resolution. Some of the features so far include:
- K/D Tree collision detection broadphase
- CCD scheme loosely based on the Frostbite GDC presentation "Cloth Self Collision with Predictive Contacts".
- Projected Gauss-Seidel solver
- Conjugate Gradient Descent solver
- Distance and non-penetration constraints for both solvers
- Debug tools for pausing / stepping through the simulation, tweaking solver params etc. Visualization of edge-edge and point-triangle contacts

<img src="https://github.com/nithinp7/Rib-Cage/blob/main/Screenshots/SelfCollision_BVH.gif" height=450/>
