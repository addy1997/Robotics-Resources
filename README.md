# Robotics-Resources
A curated list of libraries, softwares, simulators for robotics. 


![bubble](https://github.com/addy1997/Robotics-Resources/blob/master/robot.gif)


[![Software License](https://img.shields.io/badge/license-MIT-brightgreen.svg)](LICENSE)  [![Build Status](https://ci.appveyor.com/api/projects/status/8e784doc5sye7c41?svg=true)](https://ci.appveyor.com/project/addy1997/Robotics-Resources) [![Stars](https://img.shields.io/github/stars/addy1997/Robotics-Resources.svg?style=flat&label=Star&maxAge=86400)](STARS)  [![Contributions](https://img.shields.io/github/commit-activity/m/addy1997/Robotics-Resources.svg?color=%09%2346c018)](https://github.com/addy1997/Robotics-Resources/graphs/commit-activity)




#### Note: The data in this repository is updated and accurate to the best of my knowledge. In case of any correction, deletion  or addition of information please raise an issue. A mail can also be written to [adwaitnaik2@gmail.com].


#### Table of Contents
* [Libraries](#libraries)
* [Visualization](#Visualization)
* [Robot Vision](#robot-vision)
* [Robot Perception](#robot-perception)
* [Motion Planning](#Motion-planning)
* [Graphics Engine/ Environments](#graphics-engine)
* [Some cool repositories](#some-cool-repositories)
* [Other essential packages and libraries](#other-essential-packages-and-libraries)
* [Usage & Contribution](#usage-and-contribution)
* [Code Of Conduct](#code-of-conduct)


## [Libraries](#Robotics-Resources)

###### Python-based libraries

* Pybotics - python based library for roboti kinematics and calibration. It is mainly designed to work on [Denavit–Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters) principle. [[github](https://github.com/nnadeau/pybotics) ![Pybotics](https://img.shields.io/github/stars/nnadeau/pybotics.svg?style=flat&label=Star&maxAge=86400)]

* PyBullet - a python module for physics simulation of robots, loading URDF, SDF, MJCF files etc.[[github](https://github.com/bulletphysics/bullet3) ![PyBullet](https://img.shields.io/github/stars/bulletphysics/bullet3.svg?style=flat&label=Star&maxAge=86400)] 

* Pygame - a library for used for game development and creating objects in 2D space.[[github](https://github.com/pygame/pygame)  ![Pygame](https://img.shields.io/github/stars/pygame/pygame.svg?style=flat&label=Star&maxAge=86400)]


* pclpy 0.11.0 - a library used for creating point clouds for applications related to computer vision, robotics and computer graphics. [[github](https://github.com/davidcaron/pclpy) ![pclpy 0.11.0](https://img.shields.io/github/stars/davidcaron/pclpy.svg?style=flat&label=Star&maxAge=86400)]

* collision - this library is mainly used for collision checking in 2D space along with pygame. [[github](https://github.com/qwertyquerty/collision) ![collision](https://img.shields.io/github/stars/qwertyquerty/collision.svg?style=flat&label=Star&maxAge=86400)]

* Ropy - A library by Peter Corke based on the paper "Maximising manipulability during resolved-rate motion control,".[[github](https://github.com/jhavl/ropy)![Ropy](https://img.shields.io/github/stars/jhavl/ropy.svg?style=flat&label=Star&maxAge=86400)][[Paper](https://arxiv.org/abs/2002.11901)]


###### C++ based libraries

* MRPT - Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers in mobile robotics and computer vision. [[github](https://github.com/MRPT/mrpt) ![mrpt](https://img.shields.io/github/stars/MRPT/mrpt.svg?style=flat&label=Star&maxAge=86400)]

* Flexible collision library - this is mainly used for collision checking in 3D space in motion planning. [[github](https://github.com/flexible-collision-library/fcl) ![Flexible collision library](https://img.shields.io/github/stars/flexible-collision-library/fcl.svg?style=flat&label=Star&maxAge=86400)]

* Trajopt - is a C++ based library by UC berkeley for Motion planning, trajectory optimisation, etc. [[github](https://github.com/joschu/trajopt) ![Trajopt](https://img.shields.io/github/stars/joschu/trajopt.svg?style=flat&label=Star&maxAge=86400)]


## [Visualization](#Robotics-Resources)

* VTKexamples - a library based on pyhton for collision detection, making 3D meshes etc. [[github](https://github.com/lorensen/VTKExamples) ![VTKexamples](https://img.shields.io/github/stars/lorensen/VTKExamples.svg?style=flat&label=Star&maxAge=86400)]

* Pyglet - a library primarily used for game development and computer graphics applications.[[github](https://github.com/pyglet/pyglet) ![Pyglet](https://img.shields.io/github/stars/pyglet/pyglet.svg?style=flat&label=Star&maxAge=86400)]

* PyOpenGL - a cross platform library based on python and OpenGL. [[github](https://github.com/mcfletch/pyopengl) ![PyOpenGL](https://img.shields.io/github/stars/mcfletch/pyopengl.svg?style=flat&label=Star&maxAge=86400)]

## [Robot Vision](#Robotics-Resources)

* OpenCV - OpenCV (Open Source Computer Vision Library) is an open source computer vision and machine learning software library. OpenCV was built to provide a common infrastructure for computer vision applications and to accelerate the use of machine perception in the commercial products. [[github](https://github.com/opencv/opencv)![OpenCV](https://img.shields.io/github/stars/opencv/opencv.svg?style=flat&label=Star&maxAge=86400)]

* SimpleCV - SimpleCV is an open source framework for building computer vision applications. With it, you get access to several high-powered computer vision libraries such as OpenCV – without having to first learn about bit depths, file formats, color spaces, buffer management, eigenvalues, or matrix versus bitmap storage. [[github](https://github.com/sightmachine/SimpleCV)![SimpleCV](https://img.shields.io/github/stars/sightmachine/SimpleCV.svg?style=flat&label=Star&maxAge=86400)]

## [Robot Perception](#Robotics-Resources)

* CUDA Visual Library - This library focuses on the front-end of VIO pipelines.[[github](https://github.com/uzh-rpg/vilib)![CUDA Visual Library](https://img.shields.io/github/stars/uzh-rpg/vilib.svg?style=flat&label=Star&maxAge=86400)]

## [Motion Planning](#Robotics-Resources)

* sparse-rrt 0.0.2 - This package is based on Sparse-RRT project [sparse_rrt](https://bitbucket.org/pracsys/sparse_rrt/).The main purpose of this work is to allow running Sparse-RRT planner in python environment.[[github](https://github.com/olegsinyavskiy/sparse_rrt) ![sparse_rrt](https://img.shields.io/github/stars/olegsinyavskiy/sparse_rrt.svg?style=flat&label=Star&maxAge=86400)]

* OMPL - OMPL, the Open Motion Planning Library, consists of many state-of-the-art sampling-based motion planning algorithms. OMPL itself does not contain any code related to, e.g., collision checking or visualization.[[github](https://github.com/ompl/ompl) ![OMPL](https://img.shields.io/github/stars/ompl/ompl.svg?style=flat&label=Star&maxAge=86400)]

* MPL - Motion Planning Kit (MPK) is a C++ library and toolkit for developing single- and multi-robot motion planners.[Documentaion](http://ai.stanford.edu/~mitul/mpk/)

* Robotics Library - The Robotics Library (RL) is a self-contained C++ library for robot kinematics, motion planning and control. It covers mathematics, kinematics and dynamics, hardware abstraction, motion planning, collision detection, and visualization.[Documentation](https://www.roboticslibrary.org/)

* SIMOX - The aim of the lightweight platform independent C++ toolbox Simox is to provide a set of algorithms for 3D simulation of robot systems, sampling based motion planning and grasp planning.[[github](https://github.com/softbankrobotics-research/Simox) ![SIMOX](https://img.shields.io/github/stars/softbankrobotics-research/Simox.svg?style=flat&label=Star&maxAge=86400)]

## [Graphics Engine/ Environments](#Robotics-Resources)

* OpenRave( Open Robotics Automation Virtual Environment) - it focuses on developing and testing motion planning algorithms for robotic applications. It is available in C++, Python. [Documentation](http://openrave.org/docs/latest_stable/)

* Webots - It provides a complete development environment to model, program and simulate robots, vehicles and biomechanical systems.[[github](https://github.com/cyberbotics/webots)![Webots](https://img.shields.io/github/stars/cyberbotics/webots.svg?style=flat&label=Star&maxAge=86400)]

* Stage - Stage is a 2(.5)D robotics standalone simulator and can also be used as a C++ library to build your own simulation environment. [[github](https://github.com/rtv/Stage)![Stage](https://img.shields.io/github/stars/rtv/Stage.svg?style=flat&label=Star&maxAge=86400)]

* Player - is one of the most popular mobile robot simulator. [[github](https://github.com/playerproject/player)![Player](https://img.shields.io/github/stars/playerproject/player.svg?style=flat&label=Star&maxAge=86400)]

* RoboDK - is a robot simulator which allows programming and simulating any robot online and offline. It is mailny used for industrial applications. [[github](https://github.com/RoboDK/RoboDK-API)![RoboDK](https://img.shields.io/github/stars/RoboDK/RoboDK-API.svg?style=flat&label=Star&maxAge=86400)]

* Mobile Robot Simulator - Mobile robot simulator in MATLAB.[[github](https://github.com/sjchoi86/Mobile-robot-simulator)![Mobile Robot Simulator](https://img.shields.io/github/stars/sjchoi86/Mobile-robot-simulator.svg?style=flat&label=Star&maxAge=86400)]


## [Some cool repositories](#Robotics-Resources)

## [Other essential packages and libraries](#Robotics-Resources)

* trimesh - a package for loading and making meshes. [[github](https://github.com/mikedh/trimesh)![trimesh](https://img.shields.io/github/stars/mikedh/trimesh.svg?style=flat&label=Star&maxAge=86400)]

* Tkinter - a package used for visualization. [documentation](https://wiki.python.org/moin/TkInter)  [tutorial](https://github.com/Dvlv/Tkinter-By-Example)

* Pymesh - is a python based rapid prototyping platform for geometric and computer vision applications.[[github](https://github.com/PyMesh/PyMesh)![Pymesh](https://img.shields.io/github/stars/PyMesh/PyMesh.svg?style=flat&label=Star&maxAge=86400)]

* Mesh -A Processing library for computing convex hulls, delaunay graphs and voronoi graphs from groups of points.[[github](https://github.com/leebyron/mesh) ![Mesh](https://img.shields.io/github/stars/leebyron/Mesh.svg?style=flat&label=Star&maxAge=86400)]

* OpenMesh - A generic and efficient polygon mesh data structure. [[github](https://github.com/etlapale/OpenMesh)![OpenMesh](https://img.shields.io/github/stars/etlapale/OpenMesh.svg?style=flat&label=Star&maxAge=86400)]  [[Docs](https://www.graphics.rwth-aachen.de/software/openmesh/svn/)]

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


## [Usage & Contribution](#Robotics-Resources)


## [Code Of Conduct](#Robotics-Resources)
























