# NASAJPLTensegrity Spring 2020 Capstone NTRTsim Procedure

This repository is an adaptation of an existing rendition of the NASA Tensegrity Robotics Toolkit (NTRTSim) by developer "kmorse". The included files are to be compiled by NTRTsim, creating a simulation of a 6-bar, 24-tensile member tensegrity structure with a central payload. The simulation produces both visual and numeric output (position, rotation, tension, etc) associated with each tensile member and compression member, as well as the central payload. Instructions to run the simulation and obtain the visual and numeric output data are as follows:

1. Follow these instructions to get NTRTsim working out of the box: https://ntrtsim.readthedocs.io/en/latest/setup.html#installing-ntrt-in-a-virtual-machine

2. Clone this repository into ~/NTRTsim/src/dev. You'll have to install git first in order to do it through the terminal. This is where all of the source code for each application resides.

Optional but highly recommended: install Oh My Zsh and Atom (or another text editor)
  Zsh:  https://www.howtoforge.com/tutorial/how-to-setup-zsh-and-oh-my-zsh-on-linux/
  Atom: https://flight-manual.atom.io/getting-started/sections/installing-atom/#platform-linux

3. Add "NASAJPLTensegrity" as a subdirectory in the ```CMakeLists.txt``` file in ```~/NTRTsim/src/dev```

4. That is most of the setup. The actual inputs to the tensegrity structure can be found in ```T6Model.cpp```. Update this code to update the structure. Things like length of the rods, orientation of the rods, position of the rods, initial drop height, material properties, etc can all be changed in here.

To build, navigate to ```~/NTRTsim/bin/``` and run ```./build.sh```. Do this whenever source code is changed.

5. To run, navigate to ```~/NTRTsim/build/dev/NASAJPLTensegrity``` and run ```./AppNASA_JPL_Tensegrity```.

6. The output file is placed in the location declared in line 98 of ```~/NTRTsim/src/dev/NASAJPLTensegrity/AppSUPERball.cpp```. It was most useful to output the file to a shared folder to do post processing in MATLAB on a local machine. Use this tutorial to setup a shared VirtualBox folder: https://www.youtube.com/watch?v=aodqRB38Bc4&t=1s

In order to sweep over large sets of tensegrity inputs such as pretension and stiffness, ```autorun.sh``` can be used. Move this file to ```~/NTRTsim/bin/```, and run with ```./autorun.sh```. Ensure line 32 points to the correct build location. It defaults to running the simulation for 8 seconds, so update this as necessary.

Finally, any additional post processing can be performed using the included ```NTRTsim_Post_Processing_Multi_Run.m``` script. Move this to local machine and point it towards a shared folder location.
