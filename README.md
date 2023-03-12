# Project #2 Dijkstra Algo Maze Search
Author: Doug Summerlin (dsumm1001@gmail.com, dsummerl@umd.edu)  
ENPM661 Spring 2023: Robotic Path Planning

This project is basic visual implementation of the Dijkstra algorithm for a point robot navigating a maze

FILESYSTEMS: cd into the folder titled "proj2_douglas_summerlin" after downloading it on an Ubuntu or Windows operating system. 
Nothing else should be required here, the code relies on no external files. 

Ensure the following libraries are installed so they can be imported as such:
    import numpy as np
    import matplotlib.pyplot as plt
    import cv2
    import math
    from queue import PriorityQueue
    import time

OPERATING SYSTEM: Windows 11 (but Ubuntu Linux should also work)

TO RUN SCRIPTS:   
\$ cd ~/home/.../proj2\_douglas\_summerlin.zip/  
\$ python3 dijkstra\_doug\_summerlin.py

All of this script is hardcoded so just run the script and all of the ouput video .mp4 file will be generated.

The program will begin by prompting the user for the start and goal node coordinates in x,y format. Please enter integer numbers between 5-595 and 5-245 respectively for x and y coordinates, separated only be a comma and no whitespace, eg. "595,245".

The program will use the Dijkstra algorithm to find the optimal path from the established start node to the established goal node. Once the optimal path has been found, the program will prompt:

"Yay! Goal node located... Operation took  X.X seconds."

The program will then display the path on an image window imposed on the maze, with all of the searched nodes appearing blue. Close this window to continue

The program will then prepare a visual simulation of the search process, generate the optimal pathline, and then simulate a mobile robot following the optimal path. This simulation will be generated and saved as an .mp4 file, before finally appearing as a video window on the OS GUI. Once the video is closed or the video ends, the program will terminate. 