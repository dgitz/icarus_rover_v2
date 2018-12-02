Author: David Gitz
Task: Sandbox-Compute
Support Programs
A. Test LoopTimer

Usage: This test program can run on any Linux device and is used to analyze how fast a loop can iterate, using the following methods:
 - 
  >>cd ~/catkin_ws/src/icarus_rover_v2/src/Compute/unit_tests/
  >>g++ test_looptimer.cpp -o test_looptimer

Results:
Ubuntu x86/64 PC (Linux dgitzrosmaster 3.19.0-80-generic #88~14.04.1-Ubuntu SMP Fri Jan 13 14:54:07 UTC 2017 x86_64 x86_64 x86_64 GNU/Linux)
Running Simple Loop Iterator.
Time delta: 19.9942 sec.
Running Pointer Loop Iterator.
Time delta: 14.576 sec.
Time Improvement: 27.0991%

Raspberry Pi 2 Model B+ (Linux ControlModule1 4.4.13-v7+ #894 SMP Mon Jun 13 13:13:27 BST 2016 armv7l GNU/Linux)
Running Simple Loop Iterator.
Time delta: 80.9362 sec.
Running Pointer Loop Iterator.
Time delta: 46.0937 sec.
Time Improvement: 43.0493%




