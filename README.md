 ![LOGO](https://github.com/addy1997/Robotics-Resources/blob/master/Screenshot%202020-06-28%20at%201.09.06%20PM.png)
 
 -------------------------------------------------------------------------------------------------------------------
A curated list of libraries, softwares, simulators for robotics. Recently, I added a list of important books and conferences for Robotics.


![bubble](https://github.com/addy1997/Robotics-Resources/blob/master/robot.gif)


[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](LICENSE)  [![Build Status](https://ci.appveyor.com/api/projects/status/8e784doc5sye7c41?svg=true)](https://ci.appveyor.com/project/addy1997/Robotics-Resources) [![Stars](https://img.shields.io/github/stars/addy1997/Robotics-Resources.svg?style=flat&label=Star&maxAge=86400)](STARS)  [![Contributions](https://img.shields.io/github/commit-activity/m/addy1997/Robotics-Resources.svg?color=%09%2346c018)](https://github.com/addy1997/Robotics-Resources/graphs/commit-activity)

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
#### Note: The data in this repository is updated and accurate to the best of my knowledge. In case of any correction, deletion  or addition of information please raise an issue. A mail can also be written to [adwaitnaik2@gmail.com].
-------------------------------------------------------------------------------------------------------------------------------------------------------------------

#### Table of Contents
* [Libraries](#libraries)
* [Visualization](#Visualization)
* [Robot Vision](#robot-vision)
* [Robot Perception](#robot-perception)
* [Motion Planning](#Motion-planning)
* [Graphics Engine/ Environments](#graphics-engine)
* [SLAM](#SLAM)
* [Some cool repositories](#some-cool-repositories)
* [Other essential packages and libraries](#other-essential-packages-and-libraries)
* [Textbooks](#Textbooks)
* [Scientific-journals](#Scientific-journals)
* [Series-of-international-scientific-conferences](#Series-of-international-scientific-conferences)

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Libraries](#Robotics-Resources)

###### Python-based libraries

* Pybotics - python based library for roboti kinematics and calibration. It is mainly designed to work on [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters) principle. [[github](https://github.com/nnadeau/pybotics) ![Pybotics](https://img.shields.io/github/stars/nnadeau/pybotics.svg?style=flat&label=Star&maxAge=86400)]

* PyBullet - a python module for physics simulation of robots, loading URDF, SDF, MJCF files etc.[[github](https://github.com/bulletphysics/bullet3) ![PyBullet](https://img.shields.io/github/stars/bulletphysics/bullet3.svg?style=flat&label=Star&maxAge=86400)] 

* Pygame - a library for used for game development and creating objects in 2D space.[[github](https://github.com/pygame/pygame)  ![Pygame](https://img.shields.io/github/stars/pygame/pygame.svg?style=flat&label=Star&maxAge=86400)]

* pclpy 0.11.0 - a library used for creating point clouds for applications related to computer vision, robotics and computer graphics. [[github](https://github.com/davidcaron/pclpy) ![pclpy 0.11.0](https://img.shields.io/github/stars/davidcaron/pclpy.svg?style=flat&label=Star&maxAge=86400)]

* collision - this library is mainly used for collision checking in 2D space along with pygame. [[github](https://github.com/qwertyquerty/collision) ![collision](https://img.shields.io/github/stars/qwertyquerty/collision.svg?style=flat&label=Star&maxAge=86400)]

* Ropy - A library by Peter Corke based on the paper "Maximising manipulability during resolved-rate motion control,".[[github](https://github.com/jhavl/ropy)![Ropy](https://img.shields.io/github/stars/jhavl/ropy.svg?style=flat&label=Star&maxAge=86400)][[Paper](https://arxiv.org/abs/2002.11901)]

* pybotvac - This is an unofficial API for controlling Neato Botvac Connected vacuum robots. [[github](https://github.com/stianaske/pybotvac)![pybotvac](https://img.shields.io/github/stars/stianaske/pybotvac.svg?style=flat&label=Star&maxAge=86400)]

* pybot - Research tools for autonomous systems using Python. [[github](https://github.com/spillai/pybot)![pybot](https://img.shields.io/github/stars/spillai/pybot.svg?style=flat&label=Star&maxAge=86400)]

* PyBot - a python module for creating and deploying twitter bots. [[github](https://github.com/magsol/pybot)![PyBot](https://img.shields.io/github/stars/magsol/pybot.svg?style=flat&label=Star&maxAge=86400)]

* pybotgram - A Telegram Bot written in Python with plugins based in yagop telegram-bot using tg. [[github](https://github.com/rockneurotiko/pybotgram)![pybotgram](https://img.shields.io/github/stars/rockneurotiko/pybotgram.svg?style=flat&label=Star&maxAge=86400)]

* pyrobottraining- This is a repository that can be used as a tool to teach you about programming an FRC robot using python and the RobotPy WPILib libraries.[[github](https://github.com/robotpy/pyrobottraining)![pyrobottraining](https://img.shields.io/github/stars/robotpy/pyrobottraining.svg?style=flat&label=Star&maxAge=86400)]

* airobot - AIRobot is a python library to interface with robots. It follows a similar architecture from PyRobot. [[github](https://github.com/Improbable-AI/airobot)![airobot](https://img.shields.io/github/stars/Improbable-AI/airobot.svg?style=flat&label=Star&maxAge=86400)]

* ropy - A successor to the famous Peter Corke's robotics toolbox, this library is written in python to control robots in a simulated environments. [[github](https://github.com/jhavl/ropy)![ropy](https://img.shields.io/github/stars/jhavl/ropy.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
###### C++ based libraries

* MRPT - Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers in mobile robotics and computer vision. [[github](https://github.com/MRPT/mrpt) ![mrpt](https://img.shields.io/github/stars/MRPT/mrpt.svg?style=flat&label=Star&maxAge=86400)]

* Flexible collision library - this is mainly used for collision checking in 3D space in motion planning. [[github](https://github.com/flexible-collision-library/fcl) ![Flexible collision library](https://img.shields.io/github/stars/flexible-collision-library/fcl.svg?style=flat&label=Star&maxAge=86400)]

* Trajopt - is a C++ based library by UC berkeley for Motion planning, trajectory optimisation, etc. [[github](https://github.com/joschu/trajopt) ![Trajopt](https://img.shields.io/github/stars/joschu/trajopt.svg?style=flat&label=Star&maxAge=86400)]

* Eprosima - eProsima Fast DDS implements the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium [[github](https://github.com/eProsima/Fast-DDS) ![Eprosima](https://img.shields.io/github/stars/eProsima/Fast-DDS.svg?style=flat&label=Star&maxAge=86400)]

* Sophus - This is a c++ implementation of Lie groups commonly used for 2d and 3d geometric problems (i.e. for Computer Vision or Robotics applications).[[github](https://github.com/strasdat/Sophus) ![Sophus](https://img.shields.io/github/stars/strasdat/Sophus.svg?style=flat&label=Star&maxAge=86400)]

* ModernRoboticsCpp - Mechanics, Planning, and Control C++ Library --- The primary purpose of the provided software is to be easy to read and educational, reinforcing the concepts in the book. [[github](https://github.com/Le0nX/ModernRoboticsCpp#modern-robotics--mechanics-planning-and-control) ![ModernRoboticsCpp](https://img.shields.io/github/stars/Le0nX/ModernRoboticsCpp.svg?style=flat&label=Star&maxAge=86400)]

* FIESTA: Euclidean Signed Distance Field (ESDF) is useful for online motion planning of aerial robots since it can easily query the distance_ and gradient information against obstacles. [[github](https://github.com/HKUST-Aerial-Robotics/FIESTA) ![FIESTA](https://img.shields.io/github/stars/HKUST-Aerial-Robotics/FIESTA.svg?style=flat&label=Star&maxAge=86400)]


-------------------------------------------------------------------------------------------------------------------------------------------------------------------


## [Visualization](#Robotics-Resources)

* VTKexamples - a library based on pyhton for collision detection, making 3D meshes etc. [[github](https://github.com/lorensen/VTKExamples) ![VTKexamples](https://img.shields.io/github/stars/lorensen/VTKExamples.svg?style=flat&label=Star&maxAge=86400)]

* Pyglet - a library primarily used for game development and computer graphics applications.[[github](https://github.com/pyglet/pyglet) ![Pyglet](https://img.shields.io/github/stars/pyglet/pyglet.svg?style=flat&label=Star&maxAge=86400)]

* PyOpenGL - a cross platform library based on python and OpenGL. [[github](https://github.com/mcfletch/pyopengl) ![PyOpenGL](https://img.shields.io/github/stars/mcfletch/pyopengl.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Robot Vision](#Robotics-Resources)

* OpenCV - OpenCV (Open Source Computer Vision Library) is an open source computer vision and machine learning software library. OpenCV was built to provide a common infrastructure for computer vision applications and to accelerate the use of machine perception in the commercial products. [[github](https://github.com/opencv/opencv)![OpenCV](https://img.shields.io/github/stars/opencv/opencv.svg?style=flat&label=Star&maxAge=86400)]

* SimpleCV - SimpleCV is an open source framework for building computer vision applications. With it, you get access to several high-powered computer vision libraries such as OpenCV – without having to first learn about bit depths, file formats, color spaces, buffer management, eigenvalues, or matrix versus bitmap storage. [[github](https://github.com/sightmachine/SimpleCV)![SimpleCV](https://img.shields.io/github/stars/sightmachine/SimpleCV.svg?style=flat&label=Star&maxAge=86400)]

* ViSP - Visual Servoing Platform [[github](https://github.com/lagadic/visp) ![lagadic/visp](https://img.shields.io/github/stars/lagadic/visp.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Robot Perception](#Robotics-Resources)

* CUDA Visual Library - This library focuses on the front-end of VIO pipelines.[[github](https://github.com/uzh-rpg/vilib)![CUDA Visual Library](https://img.shields.io/github/stars/uzh-rpg/vilib.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Motion Planning](#Robotics-Resources)

* sparse-rrt 0.0.2 - This package is based on Sparse-RRT project [sparse_rrt](https://bitbucket.org/pracsys/sparse_rrt/).The main purpose of this work is to allow running Sparse-RRT planner in python environment.[[github](https://github.com/olegsinyavskiy/sparse_rrt) ![sparse_rrt](https://img.shields.io/github/stars/olegsinyavskiy/sparse_rrt.svg?style=flat&label=Star&maxAge=86400)]

* OMPL - OMPL, the Open Motion Planning Library, consists of many state-of-the-art sampling-based motion planning algorithms. OMPL itself does not contain any code related to, e.g., collision checking or visualization.[[github](https://github.com/ompl/ompl) ![OMPL](https://img.shields.io/github/stars/ompl/ompl.svg?style=flat&label=Star&maxAge=86400)]

* MPL - Motion Planning Kit (MPK) is a C++ library and toolkit for developing single- and multi-robot motion planners.[Documentaion](http://ai.stanford.edu/~mitul/mpk/)

* Robotics Library - The Robotics Library (RL) is a self-contained C++ library for robot kinematics, motion planning and control. It covers mathematics, kinematics and dynamics, hardware abstraction, motion planning, collision detection, and visualization.[Documentation](https://www.roboticslibrary.org/)

* SIMOX - The aim of the lightweight platform independent C++ toolbox Simox is to provide a set of algorithms for 3D simulation of robot systems, sampling based motion planning and grasp planning.[[github](https://github.com/softbankrobotics-research/Simox) ![SIMOX](https://img.shields.io/github/stars/softbankrobotics-research/Simox.svg?style=flat&label=Star&maxAge=86400)]

* AIKIDO - Solving robotic motion planning and decision making problems. [[github](https://github.com/personalrobotics/aikido) ![aikido](https://img.shields.io/github/stars/personalrobotics/aikido.svg?style=flat&label=Star&maxAge=86400)]

* TOPP-RA - Time-parameterizing robot trajectories subject to kinematic and dynamic constraints [[github](https://github.com/hungpham2511/toppra) ![hungpham2511/toppra](https://img.shields.io/github/stars/hungpham2511/toppra.svg?style=flat&label=Star&maxAge=86400)]

* The Kautham Project - A robot simulation toolkit for motion planning [[github](https://github.com/iocroblab/kautham) ![kautham](https://img.shields.io/github/stars/iocroblab/kautham.svg?style=flat&label=Star&maxAge=86400)]

* ROS Behavior Tree - [[github](https://github.com/miccol/ROS-Behavior-Tree) ![miccol/ROS-Behavior-Tree](https://img.shields.io/github/stars/miccol/ROS-Behavior-Tree.svg?style=flat&label=Star&maxAge=86400)]

* OCS2 - Efficient continuous and discrete time optimal control implementation [[bitbucket](https://bitbucket.org/leggedrobotics/ocs2/src/master/)]

* GPMP2 - Gaussian Process Motion Planner 2 [[github](https://github.com/gtrll/gpmp2) ![gtrll/gpmp2](https://img.shields.io/github/stars/gtrll/gpmp2.svg?style=flat&label=Star&maxAge=86400)]

* Crocoddyl - Optimal control library for robot control under contact sequence [[github](https://github.com/loco-3d/crocoddyl) ![loco-3d/crocoddyl](https://img.shields.io/github/stars/loco-3d/crocoddyl.svg?style=flat&label=Star&maxAge=86400)]

* Control Toolbox - Open-Source C++ Library for Robotics, Optimal and Model Predictive Control [[github](https://github.com/ethz-adrl/control-toolbox) ![ethz-adrl/control-toolbox](https://img.shields.io/github/stars/ethz-adrl/control-toolbox.svg?style=flat&label=Star&maxAge=86400)]
* 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Graphics Engine/ Environments](#Robotics-Resources)

* OpenRave( Open Robotics Automation Virtual Environment) - it focuses on developing and testing motion planning algorithms for robotic applications. It is available in C++, Python. [Documentation](http://openrave.org/docs/latest_stable/)

* Webots - It provides a complete development environment to model, program and simulate robots, vehicles and biomechanical systems.[[github](https://github.com/cyberbotics/webots)![Webots](https://img.shields.io/github/stars/cyberbotics/webots.svg?style=flat&label=Star&maxAge=86400)]

* Stage - Stage is a 2(.5)D robotics standalone simulator and can also be used as a C++ library to build your own simulation environment. [[github](https://github.com/rtv/Stage)![Stage](https://img.shields.io/github/stars/rtv/Stage.svg?style=flat&label=Star&maxAge=86400)]

* Player - is one of the most popular mobile robot simulator. [[github](https://github.com/playerproject/player)![Player](https://img.shields.io/github/stars/playerproject/player.svg?style=flat&label=Star&maxAge=86400)]

* RoboDK - is a robot simulator which allows programming and simulating any robot online and offline. It is mailny used for industrial applications. [[github](https://github.com/RoboDK/RoboDK-API)![RoboDK](https://img.shields.io/github/stars/RoboDK/RoboDK-API.svg?style=flat&label=Star&maxAge=86400)]

* Mobile Robot Simulator - Mobile robot simulator in MATLAB.[[github](https://github.com/sjchoi86/Mobile-robot-simulator)![Mobile Robot Simulator](https://img.shields.io/github/stars/sjchoi86/Mobile-robot-simulator.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [SLAM](#Robotics-Resources)

* AprilSAM - Real-time smoothing and mapping [[github](https://github.com/xipengwang/AprilSAM) ![xipengwang/AprilSAM](https://img.shields.io/github/stars/xipengwang/AprilSAM.svg?style=flat&label=Star&maxAge=86400)]

* Cartographer -  Real-time SLAM in 2D and 3D across multiple platforms and sensor configurations [[github](https://github.com/googlecartographer/cartographer) ![cartographer](https://img.shields.io/github/stars/googlecartographer/cartographer.svg?style=flat&label=Star&maxAge=86400)]

* DSO - Novel direct and sparse formulation for Visual Odometry [[github](https://github.com/JakobEngel/dso) ![dso](https://img.shields.io/github/stars/JakobEngel/dso.svg?style=flat&label=Star&maxAge=86400)]

* ElasticFusion - Real-time dense visual SLAM system [[github](http://github.com/mp3guy/ElasticFusion) ![ElasticFusion](https://img.shields.io/github/stars/mp3guy/ElasticFusion.svg?style=flat&label=Star&maxAge=86400)]

* fiducials - Simultaneous localization and mapping using fiducial markers [[github](http://github.com/UbiquityRobotics/fiducials) ![UbiquityRobotics/fiducials](https://img.shields.io/github/stars/UbiquityRobotics/fiducials.svg?style=flat&label=Star&maxAge=86400)]

* GTSAM - Smoothing and mapping (SAM) in robotics and vision [[github](http://github.com/borglab/gtsam) ![borglab/gtsam](https://img.shields.io/github/stars/borglab/gtsam.svg?style=flat&label=Star&maxAge=86400)]

* Kintinuous - Real-time large scale dense visual SLAM system [[github](http://github.com/mp3guy/Kintinuous) ![Kintinuous](https://img.shields.io/github/stars/mp3guy/Kintinuous.svg?style=flat&label=Star&maxAge=86400)]

* LSD-SLAM - Real-time monocular SLAM [[github](http://github.com/tum-vision/lsd_slam) ![lsdslam](https://img.shields.io/github/stars/tum-vision/lsd_slam.svg?style=flat&label=Star&maxAge=86400)]

* ORB-SLAM2 - Real-time SLAM library for Monocular, Stereo and RGB-D cameras [[github](http://github.com/raulmur/ORB_SLAM2) ![ORB_SLAM2](https://img.shields.io/github/stars/raulmur/ORB_SLAM2.svg?style=flat&label=Star&maxAge=86400)]

* RTAP-Map - RGB-D Graph SLAM approach based on a global Bayesian loop closure detector [[github](http://github.com/introlab/rtabmap) ![introlab/rtabmap](https://img.shields.io/github/stars/introlab/rtabmap.svg?style=flat&label=Star&maxAge=86400)]

* SRBA - Solving SLAM/BA in relative coordinates with flexibility for different submapping strategies [[github](http://github.com/MRPT/srba) ![srba](https://img.shields.io/github/stars/MRPT/srba.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------

## [Other essential packages and libraries](#Robotics-Resources)

* ROS - ROS is a meta-operating system for your robot.  It provides language-independent and network-transparent communication for adistributed robot control system. [[github](https://github.com/ros/ros)![Ros](https://img.shields.io/github/stars/ros/ros.svg?style=flat&label=Star&maxAge=86400))]

* trimesh - a package for loading and making meshes. [[github](https://github.com/mikedh/trimesh)![trimesh](https://img.shields.io/github/stars/mikedh/trimesh.svg?style=flat&label=Star&maxAge=86400)]

* Tkinter - a package used for visualization. [documentation](https://wiki.python.org/moin/TkInter)  [tutorial](https://github.com/Dvlv/Tkinter-By-Example)

* Pymesh - is a python based rapid prototyping platform for geometric and computer vision applications.[[github](https://github.com/PyMesh/PyMesh)![Pymesh](https://img.shields.io/github/stars/PyMesh/PyMesh.svg?style=flat&label=Star&maxAge=86400)]

* Mesh -A Processing library for computing convex hulls, delaunay graphs and voronoi graphs from groups of points.[[github](https://github.com/leebyron/mesh) ![Mesh](https://img.shields.io/github/stars/leebyron/Mesh.svg?style=flat&label=Star&maxAge=86400)]

* OpenMesh - A generic and efficient polygon mesh data structure. [Relocated-to-this](https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh)  [[Docs](https://www.graphics.rwth-aachen.de/software/openmesh/svn/)]

* libigl - A simple C++ geometry processing library[[github](https://github.com/libigl/libigl)![libigl](https://img.shields.io/github/stars/libigl/libigl.svg?style=flat&label=Star&maxAge=86400)]

* PyGEL - Python library for geometry processing [[github](https://github.com/janba/GEL)
![PyGEL](https://img.shields.io/github/stars/janba/GEL.svg?style=flat&label=Star&maxAge=86400)]   [[Docs](http://www2.compute.dtu.dk/projects/GEL/PyGEL/)]

* pmp-library - C++ based library for processing and generating meshes. [[github](https://github.com/pmp-library/pmp-library/)![pmp-library](https://img.shields.io/github/stars/pmp-library/pmp-library.svg?style=flat&label=Star&maxAge=86400)]

* Delynoi: an object-oriented C++ library for the generation of polygonal meshes [[github](https://github.com/cemcen/Delynoi)![Delynoi](https://img.shields.io/github/stars/cemcen/Delynoi.svg?style=flat&label=Star&maxAge=86400)]

* Cinolib: a Generic Programming Header Only C++ Library for Processing Polygonal and Polyhedral Meshes[[github](https://github.com/mlivesu/cinolib)![Cinolib](https://img.shields.io/github/stars/mlivesu/cinolib.svg?style=flat&label=Star&maxAge=86400)]  [paper](https://www.researchgate.net/publication/332496897_Cinolib_a_Generic_Programming_Header_Only_C_Library_for_Processing_Polygonal_and_Polyhedral_Meshes)

* cilantro - A lean C++ library for working with point cloud data.[[github](https://github.com/kzampog/cilantro)![cilnatro](https://img.shields.io/github/stars/kzampog/cilantro.svg?style=flat&label=Star&maxAge=86400)]

* PDAL - Point Data Abstraction Library, is a C++ BSD library for translating and manipulating point cloud data. It is very much like the GDAL library which handles raster and vector data.[[github](https://github.com/PDAL/PDAL)![PDAL](https://img.shields.io/github/stars/PDAL/PDAL.svg?style=flat&label=Star&maxAge=86400)]

* LEPCC - Limited Error Point Cloud Compression, a library written in C++ for point cloud compression.[[github](https://github.com/PDAL/lepcc)![LEPCC](https://img.shields.io/github/stars/PDAL/lepcc.svg?style=flat&label=Star&maxAge=86400)]

* chainer - flexible framework for working with neural networks. [[github](https://github.com/chainer/chainer)![chainer](https://img.shields.io/github/stars/chainer/chainer.svg?style=flat&label=Star&maxAge=86400)]

* shogun - a machine learning toolbox build on C++. It also supports python. [[github](https://github.com/shogun-toolbox/shogun)![shogun](https://img.shields.io/github/stars/shogun-toolbox/shogun.svg?style=flat&label=Star&maxAge=86400)]

* Apache Spark - Spark is a unified analytics engine for large-scale data processing. [[github](https://github.com/apache/spark)![Apche Spark](https://img.shields.io/github/stars/apache/spark.svg?style=flat&label=Star&maxAge=86400)]

* Pinocchio - instantiates the state-of-the-art Rigid Body Algorithms for poly-articulated systems based on revisited Roy Featherstone's algorithms. Besides, Pinocchio provides the analytical derivatives of the main Rigid-Body Algorithms like the Recursive Newton-Euler Algorithm or the Articulated-Body Algorithm. [[github](https://github.com/stack-of-tasks/pinocchio)![Pinocchio](https://img.shields.io/github/stars/stack-of-tasks/Pinocchio.svg?style=flat&label=Star&maxAge=86400)]

* Crocoddyl - Contact RObot COntrol by Differential DYnamic programming Library. [[github](https://github.com/loco-3d/crocoddyl)![Crocoddyl](https://img.shields.io/github/stars/loco-3d/crocoddyl.svg?style=flat&label=Star&maxAge=86400)]

* FCL- FCL is a library for performing three types of proximity queries on a pair of geometric models composed of triangles.. [[github](https://github.com/flexible-collision-library/fcl)![FCL](https://img.shields.io/github/stars/flexible-collision-library/fcl.svg?style=flat&label=Star&maxAge=86400)]

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
## [Textbooks](#Robotics-Resources)

* [J. Angeles, Fundamentals of Robotic Mechanical Systems: Theory, Meth- ods, and Algorithms, Springer-Verlag, New York, 1997.](https://www.springer.com/gp/book/9780387224589)

* [H. Asada, J.-J.E. Slotine, Robot Analysis and Control, Wiley, New York, 1986.](https://www.wiley.com/en-in/Robot+Analysis+and+Control-p-9780471830290)

* [G.A. Bekey, Autonomous Robots, MIT Press, Cambridge, MA, 2005.](https://mitpress.mit.edu/books/autonomous-robots)

* [C. Canudas de Wit, B. Siciliano, G. Bastin, (Eds.), Theory of Robot Control, Springer-Verlag, London, 1996.](https://www.springer.com/gp/book/9781447115038)

* [J.J. Craig, Introduction to Robotics: Mechanics and Control, 3rd ed., Pearson Prentice Hall, Upper Saddle River, NJ, 2004.](https://link.springer.com/chapter/10.1007/978-1-84628-642-1_1)

* [A.J. Critchlow, Introduction to Robotics, Macmillan, New York, 1985.](https://www.worldcat.org/title/introduction-to-robotics/oclc/635623654)

* [J.F. Engelberger, Robotics in Practice, Amacom, New York, 1980.](https://www.springer.com/gp/book/9780850386691)

* [J.F. Engelberger, Robotics in Service, MIT Press, Cambridge, MA, 1989.](https://mitpress.mit.edu/books/robotics-service)

* [K.S. Fu, R.C. Gonzalez, C.S.G. Lee, Robotics: Control, Sensing, Vision,and Intelligence, McGraw-Hill, New York, 1987.](https://www.worldcat.org/title/robotics-control-sensing-vision-and-intelligence/oclc/13358751)

* [W. Khalil, E. Dombre, Modeling, Identification and Control of Robots, Hermes Penton Ltd, London, 2002.](https://www.worldcat.org/title/modeling-identification-control-of-robots/oclc/264997387)

* [A.J. Koivo, Fundamentals for Control of Robotic Manipulators, Wiley,New York, 1989.](https://www.amazon.com/Fundamentals-Control-Robotic-Manipulators-Antti/dp/0471857149)

* [Y. Koren, Robotics for Engineers, McGraw-Hill, New York, 1985.](https://catalogue.nla.gov.au/Record/4556838)

* [F.L. Lewis, C.T. Abdallah, D.M. Dawson, Control of Robot Manipulators,Macmillan, New York, 1993.](https://www.worldcat.org/title/control-of-robot-manipulators/oclc/26012747)

* [P.J. McKerrow, Introduction to Robotics, Addison-Wesley, Sydney, Australia, 1991.](https://dl.acm.org/doi/book/10.5555/532713)

* [R.M. Murray, Z. Li, S.S. Sastry, A Mathematical Introduction to Robotic Manipulation, CRC Press, Boca Raton, FL, 1994.](https://books.google.fr/books?id=D_PqGKRo7oIC)

* [S.B. Niku, Introduction to Robotics: Analysis, Systems, Applications,Prentice-Hall, Upper Saddle River, NJ, 2001.](https://www.academia.edu/40360855/Introduction_To_Robotics_Analysis_Control_Applications_2nd_Ed)

* [R.P. Paul, Robot Manipulators: Mathematics, Programming, and Control MIT Press, Cambridge, MA, 1981.](https://mitpress.mit.edu/contributors/richard-p-paul)

* [R.J. Schilling, Fundamentals of Robotics: Analysis and Control, Prentice Hall, Englewood Cliffs, NJ, 1990.](https://www.worldcat.org/title/fundamentals-of-robotics-analysis-and-control/oclc/472851222)

* [L. Sciavicco, B. Siciliano, Model ling and Control of Robot Manipulators ,2nd ed., Springer, London, UK, 2000.](https://www.springer.com/gp/book/9781852332211)

* [W.E. Snyder, Industrial Robots: Computer Interfacing and Control, Prentice-Hall, Englewood Cliffs, NJ, 1985.](https://www.amazon.in/Industrial-Robots-Interfacing-Prentice-Hall-industrial/dp/0134631595)


* [M.W. Spong, S. Hutchinson, M. Vidyasagar, Robot Modeling and Control, Wiley, New York, 2006.](https://www.wiley.com/en-us/Robot+Modeling+and+Control-p-9780471649908)

* [M. Vukobratovi ́c, Introduction to Robotics, Springer-Verlag, Berlin, Ger- many, 1989.](https://www.springer.com/gp/book/9783642829994)

* [T. Yoshikawa, Foundations of Robotics, MIT Press, Boston, MA, 1990.](https://www.tandfonline.com/doi/abs/10.1080/00207549108948075)

-------------------------------------------------------------------------------------------------------------------------------------------------------------------
## [Scientific journals](#Robotics-Resources)

* [Advanced Robotics](https://www.rsj.or.jp/en/pub/ar/about.html)
* [Autonomous Robots](https://www.springer.com/journal/10514)
* [IEEE Robotics and Automation Magazine](https://www.ieee-ras.org/publications/ram)
* [IEEE Transactions on Robotics](https://www.ieee-ras.org/publications/t-ro)
* [International Journal of Robotics Research](https://journals.sagepub.com/home/ijr)
* [Journal of Field Robotics](https://onlinelibrary.wiley.com/journal/15564967)
* [Journal of Intelligent and Robotic Systems](https://www.springer.com/journal/10846)
* [Robotica](https://www.cambridge.org/core/journals/robotica)
* [Robotics and Autonomous Systems](https://www.journals.elsevier.com/robotics-and-autonomous-systems)
* [Frontiers in Robotics and AI](https://www.frontiersin.org/journals/robotics-and-ai)
* [International Journal of Social Robotics](https://www.springer.com/journal/12369)
* [Soft Robotics](https://www.scijournal.org/impact-factor-of-soft-robotics.shtml)
* [Science Robotics](https://robotics.sciencemag.org/) 
* [Robotics and Computer-Integrated Manufacturing](https://www.google.com/search?client=firefox-b-d&q=Robotics+and+Computer-Integrated+Manufacturing)
* [EEE Robotics and Automation Letters](https://ieeexplore.ieee.org/xpl/RecentIssue.jsp?punumber=7083369)
* [IEEE/RSJ International Conference on Intelligent Robots and Systems](https://www.ieee-ras.org/conferences-workshops/financially-co-sponsored/iros)
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
## [Series of international scientific conferences](#Robotics-Resources)

* [IEEE International Conference on Robotics and Automation](https://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra)
* [IEEE/RSJ International Conference on Intelligent Robots and Systems](https://www.ieee-ras.org/component/rseventspro/event/1714-iros-2020-international-conference-on-intelligent-robots-and-systems)
* [International Conference on Advanced Robotics](https://www.ieee-ras.org/component/rseventspro/event/1592-icar-2019-international-conference-on-advanced-robotics)
* [International Symposium of Robotics Research](http://www.isrr2019.org/)
* [International Symposium on Experimental Robotics](https://link.springer.com/conference/iser)
* [Robotics: Science and Systems](https://roboticsconference.org/)
* [ACM/IEEE International Conference on Human Robot Interaction](https://dl.acm.org/conference/hri)
* [Robotics: Science and Systems](https://roboticsconference.org/)


























