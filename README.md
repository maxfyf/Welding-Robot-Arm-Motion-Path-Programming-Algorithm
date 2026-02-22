# README

### Brief Introduction

*Welding Robot Arm Motion Path Programming Algorithm* is a C++ project specifically designed for scientific research on the optimization of algorithms that help find ideal motion path of welding robot arms. This project is focused on improving the ultimate result of Beam search algorithm and Rapidly-exploring Random Tree (RRT) algorithm. In other words, it is dedicated to coming up with optimized Beam search algorithm and optimized RRT algorithm that compute outputs indicating lower distance cost for a welding robot arm, but in the meantime do not take an unacceptably long running time. Both algorithm codes and experiment codes are included in the project, and some plotting codes are involved as well.

### Project Hierarchy

Welding-Robot-Arm-Motion-Path-Programming-Algorithm

|——Test.cpp: the file involving the main function

|——global.h: the header file involving important global variables and statements of global functions

|——global.cpp: the file involving implementations of global functions

|——log.h: the header file involving statements of log functions

|——log.cpp: the file involving implementations of log functions

|——Beam.h: the header file involving important variables and structures of Beam search algorithm as well as statements of functions related to Beam search algorithm

|——Beam.cpp: the file involving implementations of functions related to Beam search algorithm

|——RRT.h: the header file involving important variables and structures of RRT algorithm as well as statements of functions related to RRT algorithm

|——RRT.cpp: the file involving implementations of functions related to RRT algorithm

|——tests.h: the header file involving statements of functions for experiments

|——tests.cpp: the file involving implementations of functions for experiments

|——tools

|——|——matplotlibcpp.h: the C++ header file providing APIs of Python's Matplotlib functions

|——|——python314.dll: auxiliary file supporting the linkage of Python's library

|——README.md: this file

### Requirements

This project requires C++ environment and basic STL libraries. You can run this project by compiling all the files and run Test.cpp. You can also modify the main function to recur the results of certain experiments as you will. If you want to call functions with visualization of the result, you need to configure the plotting file matplotlibcpp.h properly, and the header file is provided under the path '/tools'.

