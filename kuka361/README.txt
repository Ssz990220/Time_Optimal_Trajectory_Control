***************************************************************************
  tag: Diederik Verscheure  vr aug 31 07:10:07 CEST 2007  phd/projects/time_optimal_trajectory/matlab

                           phd/projects/time_optimal_trajectory/matlab -  description
                           ----------------------------
    begin                : vr augustus 31 2007
    copyright            : (C) 2007 K.U.Leuven
    email                : diederik <dot> verscheure <at> mech <dot> kuleuven <dot> be

***************************************************************************
Purpose
---------------------------------------------------------------------------
The following set of matlab files calculates the time-optimal trajectory
along a given path for the 6-DOF KUKA 361 manipulator.

Getting started
---------------
- Make sure all requirements are installed (see below).
- Start up Matlab and go to the directory where this archive was unpacked.
- Type 'makekuka361' at the Matlab prompt to compile all mex files.
  There may appear some warnings about the gcc version if gcc is used.
  Usually, these warnings can be safely ignored.
- Type 'kuka361' to start the trajectory calculation.
- A number of options can be changed in kuka361.m by commenting/uncommenting
  the appropriate lines.



Content:
---------
kuka361.m -- main file.
params361.m -- KUKA 361 file which defines a number of parameters.
makekuka361.m -- compiles all mex files.
invkinkuka361.m -- inverse kinematics.
fwkinkuka361.m -- forward kinematics.
invkin361corr.m -- inverse kinematics (calls mex file but changes axis conventions).
eeframe361corr.m -- forward kinematics (calls mex file but changes axis conventions).
invdynkuka361.m -- inverse dynamic model.
src/mass361.c -- mass matrix.
src/pseudocoriolis361.c -- coriolis matrix.
src/pseudogravity361.c -- gravity vector matrix.
src/invkin361.c -- inverse kinematics (with old convention).
src/eeframe361.c -- forward kinematics (with old convention).
optecpathcart.m -- optec path in operational space coordinates.
optecpath.m -- optec path in joint space coordinates.
timeoptpathconstr.m -- implementation of the trajectory planning algorithm with torques as variables.
timeoptpathconstropt.m -- faster implementation of the trajectory planning algorithm with torques eliminated.
timeoptpathconstrsed.m -- faster implementation of the trajectory planning algorithm with torques eliminated
                          which does not use YALMIP.
dynpathparams.m -- calculates all path-dependent quantities which are passed to timeoptpathconstr*.m.
loadsvg.m -- general purpose. Loads svg file.
setunits.m -- general purpose. Sets units on figures.
setfigtempl.m -- general purpose. Sets line width and font sizes.
savepls.m -- general purpose. Saves figures.
savefig.m -- general purpose. Saves figures.
inversT.m -- general purpose. Calculates inverse of homogeneous transformation matrix.
htranslate.m -- general purpose. Calculates homogeneous transformation matrix for a given translation.
rotx.m -- general purpose. Calculates homogeneous transformation matrix for a given rotation around the x-axis.
images/optec2.svg -- the SVG file from which the optec path is loaded.

Requirements:
-------------
YALMIP - http://www.control.isy.liu.se/~johanl
SeDuMi - http://sedumi.mcmaster.ca/

Changes:
--------
20/10/08 - Can be used without YALMIP.
11/06/08 - Fixed an issue with the spline fitting.
18/04/08 - Removed dependency on Spline toolbox.
           Added fwkinkuka361.m

***************************************************************************


