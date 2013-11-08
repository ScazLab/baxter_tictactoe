#!/bin/bash

#Smoothly from home to bottom heap
cat about_over_heap.raw > grasp.rawt
cat over_heap.raw >> grasp.rawt
cat bottom_heap.raw >> grasp.rawt

#Mechanistic from home to bottom heap
cat grasp_mech_1.raw > mech_grasp.rawt
cat grasp_mech_1.raw >> mech_grasp.rawt
cat grasp_mech_2.raw >> mech_grasp.rawt
cat grasp_mech_2.raw >> mech_grasp.rawt
cat over_heap.raw >> mech_grasp.rawt
cat over_heap.raw >> mech_grasp.rawt
cat bottom_heap.raw >> mech_grasp.rawt

#Smoothly from bottom heap to home
cat over_heap.raw > back@0x0.rawt
cat about_over_heap.raw >> back@0x0.rawt
cat home.raw >> back@0x0.rawt
cat home.raw >> back@0x0.rawt

#Mechanistic from bottom heap to home
cat over_heap.raw > mech_back@0x0.rawt
cat over_heap.raw >> mech_back@0x0.rawt
cat grasp_mech_2.raw >> mech_back@0x0.rawt
cat grasp_mech_2.raw >> mech_back@0x0.rawt
cat grasp_mech_1.raw >> mech_back@0x0.rawt
cat grasp_mech_1.raw >> mech_back@0x0.rawt
cat home.raw >> mech_back@0x0.rawt
cat home.raw >> mech_back@0x0.rawt


#Smoothly from bottom heap to cell 1x1
cat over_heap.raw > put@1x1.rawt
cat over_1x1.raw >> put@1x1.rawt
cat bottom_1x1.raw >> put@1x1.rawt

#Mechanistic from bottom heap to cell 1x1
cat over_heap.raw > mech_put@1x1.rawt
cat over_heap.raw >> mech_put@1x1.rawt
cat over_3x1.raw >> mech_put@1x1.rawt
cat over_3x1.raw >> mech_put@1x1.rawt
cat over_2x1.raw >> mech_put@1x1.rawt
cat over_1x1.raw >> mech_put@1x1.rawt
cat over_1x1.raw >> mech_put@1x1.rawt
cat bottom_1x1.raw >> mech_put@1x1.rawt

#Smoothly from cell 1x1  to home
cat over_1x1.raw > back@1x1.rawt
cat back@1x1_1.raw >> back@1x1.rawt
cat home.raw >> back@1x1.rawt
cat home.raw >> back@1x1.rawt

#Mechanistic from cell 1x1  to home
cat over_1x1.raw > mech_back@1x1.rawt
cat over_1x1.raw >> mech_back@1x1.rawt
cat over_2x1.raw >> mech_back@1x1.rawt
cat over_3x1.raw >> mech_back@1x1.rawt
cat over_3x1.raw >> mech_back@1x1.rawt
cat mech_back@0x0.rawt >> mech_back@1x1.rawt


#Smoothly from bottom heap to cell 1x2
cat over_heap.raw > put@1x2.rawt
cat over_1x2.raw >> put@1x2.rawt
cat bottom_1x2.raw >> put@1x2.rawt

#Mechanistic from bottom heap to cell 1x2
cat over_heap.raw > mech_put@1x2.rawt
cat over_heap.raw >> mech_put@1x2.rawt
cat over_3x1.raw >> mech_put@1x2.rawt
cat over_3x1.raw >> mech_put@1x2.rawt
cat over_2x1.raw >> mech_put@1x2.rawt
cat over_1x1.raw >> mech_put@1x2.rawt
cat over_1x1.raw >> mech_put@1x2.rawt
cat over_1x2.raw >> mech_put@1x2.rawt
cat over_1x2.raw >> mech_put@1x2.rawt
cat bottom_1x2.raw >> mech_put@1x2.rawt

#Smoothly from cell 1x2 to home
cat over_1x2.raw > back@1x2.rawt
cat back@1x1_1.raw >> back@1x2.rawt
cat home.raw >> back@1x2.rawt
cat home.raw >> back@1x2.rawt

#Mechanistic from cell 1x2  to home
cat over_1x2.raw > mech_back@1x2.rawt
cat over_1x2.raw >> mech_back@1x2.rawt
cat over_1x1.raw >> mech_back@1x2.rawt
cat over_1x1.raw >> mech_back@1x2.rawt
cat over_2x1.raw >> mech_back@1x2.rawt
cat over_3x1.raw >> mech_back@1x2.rawt
cat over_3x1.raw >> mech_back@1x2.rawt
cat mech_back@0x0.rawt >> mech_back@1x2.rawt


#Smoothly from bottom heap to cell 1x3
cat over_heap.raw > put@1x3.rawt
cat over_1x3.raw >> put@1x3.rawt
cat bottom_1x3.raw >> put@1x3.rawt

#Mechanistic from bottom heap to cell 1x3
cat over_heap.raw > mech_put@1x3.rawt
cat over_heap.raw >> mech_put@1x3.rawt
cat over_3x1.raw >> mech_put@1x3.rawt
cat over_3x1.raw >> mech_put@1x3.rawt
cat over_2x1.raw >> mech_put@1x3.rawt
cat over_1x1.raw >> mech_put@1x3.rawt
cat over_1x1.raw >> mech_put@1x3.rawt
cat over_1x2.raw >> mech_put@1x3.rawt
cat over_1x3.raw >> mech_put@1x3.rawt
cat over_1x3.raw >> mech_put@1x3.rawt
cat bottom_1x3.raw >> mech_put@1x3.rawt

#Smoothly from cell 1x3 to home
cat over_1x3.raw > back@1x3.rawt
cat back@1x1_1.raw >> back@1x3.rawt
cat home.raw >> back@1x3.rawt
cat home.raw >> back@1x3.rawt

#Mechanistic from cell 1x3 to home
cat over_1x3.raw > mech_back@1x3.rawt
cat over_1x3.raw >> mech_back@1x3.rawt
cat over_1x2.raw >> mech_back@1x3.rawt
cat over_1x1.raw >> mech_back@1x3.rawt
cat over_1x1.raw >> mech_back@1x3.rawt
cat over_2x1.raw >> mech_back@1x3.rawt
cat over_3x1.raw >> mech_back@1x3.rawt
cat over_3x1.raw >> mech_back@1x3.rawt
cat mech_back@0x0.rawt >> mech_back@1x3.rawt


#Smoothly from bottom heap to cell 2x1
cat over_heap.raw > put@2x1.rawt
cat over_2x1.raw >> put@2x1.rawt
cat bottom_2x1.raw >> put@2x1.rawt

#Mechanistic from bottom heap to cell 2x1
cat over_heap.raw > mech_put@2x1.rawt
cat over_heap.raw >> mech_put@2x1.rawt
cat over_3x1.raw >> mech_put@2x1.rawt
cat over_3x1.raw >> mech_put@2x1.rawt
cat over_2x1.raw >> mech_put@2x1.rawt
cat over_2x1.raw >> mech_put@2x1.rawt
cat bottom_2x1.raw >> mech_put@2x1.rawt

#Smoothly from cell 2x1 to home
cat over_2x1.raw > back@2x1.rawt
cat back@1x1_1.raw >> back@2x1.rawt
cat home.raw >> back@2x1.rawt
cat home.raw >> back@2x1.rawt

#Mechanistic from cell 2x1 to home
cat over_2x1.raw > mech_back@2x1.rawt
cat over_2x1.raw >> mech_back@2x1.rawt
cat over_3x1.raw >> mech_back@2x1.rawt
cat over_3x1.raw >> mech_back@2x1.rawt
cat mech_back@0x0.rawt >> mech_back@2x1.rawt


#Smoothly from bottom heap to cell 2x2
cat over_heap.raw > put@2x2.rawt
cat over_2x2.raw >> put@2x2.rawt
cat bottom_2x2.raw >> put@2x2.rawt

#Mechanistic from bottom heap to cell 2x2
cat over_heap.raw > mech_put@2x2.rawt
cat over_heap.raw >> mech_put@2x2.rawt
cat over_3x1.raw >> mech_put@2x2.rawt
cat over_3x2.raw >> mech_put@2x2.rawt
cat over_3x2.raw >> mech_put@2x2.rawt
cat over_2x2.raw >> mech_put@2x2.rawt
cat over_2x2.raw >> mech_put@2x2.rawt
cat bottom_2x2.raw >> mech_put@2x2.rawt

#Smoothly from cell 2x2 to home
cat over_2x2.raw > back@2x2.rawt
cat back@1x1_1.raw >> back@2x2.rawt
cat home.raw >> back@2x2.rawt
cat home.raw >> back@2x2.rawt

#Mechanistic from cell 2x2 to home
cat over_2x2.raw > mech_back@2x2.rawt
cat over_2x2.raw >> mech_back@2x2.rawt
cat over_2x1.raw >> mech_back@2x2.rawt
cat over_2x1.raw >> mech_back@2x2.rawt
cat over_3x1.raw >> mech_back@2x2.rawt
cat over_3x1.raw >> mech_back@2x2.rawt
cat mech_back@0x0.rawt >> mech_back@2x2.rawt

#Smoothly from bottom heap to cell 2x3
cat over_heap.raw > put@2x3.rawt
cat over_2x3.raw >> put@2x3.rawt
cat bottom_2x3.raw >> put@2x3.rawt

#Mechanistic from bottom heap to cell 2x3
cat over_heap.raw > mech_put@2x3.rawt
cat over_heap.raw >> mech_put@2x3.rawt
cat over_3x1.raw >> mech_put@2x3.rawt
cat over_3x1.raw >> mech_put@2x3.rawt
cat over_2x1.raw >> mech_put@2x3.rawt
cat over_2x1.raw >> mech_put@2x3.rawt
cat over_2x2.raw >> mech_put@2x3.rawt
cat over_2x3.raw >> mech_put@2x3.rawt
cat over_2x3.raw >> mech_put@2x3.rawt
cat bottom_2x3.raw >> mech_put@2x3.rawt

#Smoothly from cell 2x3 to home
cat over_2x3.raw > back@2x3.rawt
cat back@1x1_1.raw >> back@2x3.rawt
cat home.raw >> back@2x3.rawt
cat home.raw >> back@2x3.rawt

#Mechanistic from cell 2x3  to home
cat over_2x3.raw > mech_back@2x3.rawt
cat over_2x3.raw >> mech_back@2x3.rawt
cat over_2x2.raw >> mech_back@2x3.rawt
cat over_2x1.raw >> mech_back@2x3.rawt
cat over_2x1.raw >> mech_back@2x3.rawt
cat over_3x1.raw >> mech_back@2x3.rawt
cat over_3x1.raw >> mech_back@2x3.rawt
cat mech_back@0x0.rawt >> mech_back@2x3.rawt


#Smoothly from bottom heap to cell 3x1
cat over_heap.raw > put@3x1.rawt
cat over_3x1.raw >> put@3x1.rawt
cat bottom_3x1.raw >> put@3x1.rawt

#Mechanistic from bottom heap to cell 3x1
cat over_heap.raw > mech_put@3x1.rawt
cat over_heap.raw >> mech_put@3x1.rawt
cat over_3x1.raw >> mech_put@3x1.rawt
cat over_3x1.raw >> mech_put@3x1.rawt
cat bottom_3x1.raw >> mech_put@3x1.rawt

#Smoothly from cell 3x1 to home
cat over_3x1.raw > back@3x1.rawt
cat home.raw >> back@3x1.rawt
cat home.raw >> back@3x1.rawt

#Mechanistic from cell 3x1  to home
cat over_3x1.raw > mech_back@3x1.rawt
cat over_3x1.raw >> mech_back@3x1.rawt
cat mech_back@0x0.rawt >> mech_back@3x3.rawt

#Smoothly from bottom heap to cell 3x2
cat over_heap.raw > put@3x2.rawt
cat over_3x2_1.raw >> put@3x2.rawt
cat bottom_3x2.raw >> put@3x2.rawt

#Mechanistic from bottom heap to cell 3x2
cat over_heap.raw > mech_put@3x2.rawt
cat over_heap.raw >> mech_put@3x2.rawt
cat over_3x1.raw >> mech_put@3x2.rawt
cat over_3x2_1.raw >> mech_put@3x2.rawt
cat over_3x2_1.raw >> mech_put@3x2.rawt
cat bottom_3x2.raw >> mech_put@3x2.rawt

#Smoothly from cell 3x2 to home
cat over_3x2.raw > back@3x2.rawt
cat home.raw >> back@3x2.rawt
cat home.raw >> back@3x2.rawt

#Mechanistic from cell 3x2 to home
cat over_3x2.raw > mech_back@3x2.rawt
cat over_3x2.raw >> mech_back@3x2.rawt
cat over_3x1.raw >> mech_back@3x2.rawt
cat mech_back@0x0.rawt >> mech_back@3x2.rawt


#Smoothly from bottom heap to cell 3x3
cat over_heap.raw > put@3x3.rawt
cat over_3x3.raw >> put@3x3.rawt
cat bottom_3x3.raw >> put@3x3.rawt

#Mechanistic from bottom heap to cell 3x3
cat over_heap.raw > mech_put@3x3.rawt
cat over_heap.raw >> mech_put@3x3.rawt
cat over_3x1.raw >> mech_put@3x3.rawt
cat over_3x2.raw >> mech_put@3x3.rawt
cat over_3x3.raw >> mech_put@3x3.rawt
cat over_3x3.raw >> mech_put@3x3.rawt
cat bottom_3x3.raw >> mech_put@3x3.rawt

#Smoothly from cell 3x3 to home
cat over_3x3.raw > back@3x3.rawt
cat home.raw >> back@3x3.rawt
cat home.raw >> back@3x3.rawt

#Mechanistic from cell 3x3 to home
cat over_3x3.raw > mech_back@3x3.rawt
cat over_3x3.raw >> mech_back@3x3.rawt
cat over_3x2.raw >> mech_back@3x3.rawt
cat over_3x1.raw >> mech_back@3x3.rawt
cat mech_back@0x0.rawt >> mech_back@3x3.rawt

#generating file with all trajectories
rm trajectories.xml
sh raw_trajectories_to_xml_trajectory_file.sh "*.rawt" > /dev/null
echo "WARNNING: time frames must be adapted for each point in the trajectories from trajectories.xml"

#running trajectory player with the trajectories just created
rosrun tictactoe test_trajectory_player trajectories.xml  /sdk/robot/limb/left/follow_joint_trajectory

