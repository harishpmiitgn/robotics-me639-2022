# OSAKE - 1

The project implements trajectory control and end-effector force control on SysIDEA Lab's 2R-Manipulator, OSAKE-1.

An STM32 microcontroller was used to control the robot hardware. The main code for the controller is present in `main.c`. The code for our implementation of trajectory control and end-effector force control is written between line numbers 458 and 531 of the main file.

For Task 0, we have used the script `jacobian.py` inside miscellaneous folder. For Task 1, we used the script `generate_trajectory.py` to generate circular and linear trajectory, which were copied into the STM32 main file.

The file `Midsem Project - OSAKE I.pdf` contains Task 0 as well as details on our implementation of task 1, 2 and 3. The same is explained via the video [Explainer Video.mp4](https://drive.google.com/file/d/1gCFGVo-jiljBiFM0rwVetGmGDAY7_APJ/view?usp=sharing).