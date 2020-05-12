#!/bin/bash

clear
echo "NTRTsim Auto Test Runner"
echo "Enter mesh size and increments. If you want to hold a variable constant, enter 0 for increment."

read -p "Number of steps (mesh size): " steps
read -p "Initial stiffnessouter: " s_o_initial
read -p "stiffnessouter increment: " s_o_increment
read -p "Initial stiffnessinner: " s_i_initial
read -p "stiffnessinner increment: " s_i_increment
read -p "Initial pretensionouter: " p_o_initial
read -p "pretensionouter increment: " p_o_increment
read -p "Initial pretensioninner: " p_i_initial
read -p "pretensioninner increment: " p_i_increment

# NOTE single quotes are interpreted as string literals by BASH
# double quotes allow variable expansion

# sed "s/STIFFNESSOUTER/$stiffnessouter/g" ~/SIM2/NTRTsim/src/dev/NASAJPLTensegrity/AppSUPERball_Template.cpp > ~/SIM2/NTRTsim/src/dev/NASAJPLTensegrity/AppSUPERball_TESTREPLACE.cpp
for i in $(seq 0 $steps)
do
  s_o=$(($s_o_initial + $s_o_increment*i))
  s_i=$(($s_i_initial + $s_i_increment*i))
  p_o=$(($p_o_initial + $p_o_increment*i))
  p_i=$(($p_i_initial + $p_i_increment*i))

  sed -e "s/STIFFNESSOUTER/$s_o/g" -e "s/STIFFNESSINNER/$s_i/g" -e "s/PRETENSIONINNER/$p_o/g" -e "s/PRETENSIONOUTER/$p_i/g" ~/SIM2/NTRTsim/src/dev/NASAJPLTensegrity/AppSUPERball_Template.cpp > ~/SIM2/NTRTsim/src/dev/NASAJPLTensegrity/AppSUPERball.cpp

  # build and run with 8 second timeout. Need to update this if gravity changes
  ./build.sh
  ~/NTRTsim/build/dev/NASAJPLTensegrity/AppNASA_JPL_Tensegrity & sleep 8; kill $!
done

sleep 1
echo "====================================="
echo "Done running your test :)"
echo "====================================="
