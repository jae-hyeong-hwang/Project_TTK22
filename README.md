# Project_TTK22
project of TTK22, make asv caravela search certain area
you should have installed LSTS Toolchain and have to know basic commands and how system works.
Please have a look the github page of LSTS Toolchain https://github.com/LSTS or official website https://www.lsts.pt/toolchain

if you are running in your own system, make a folder "Test" inside Maneuver.
so the directory will be   
~ dune/source/src/Maneuver/Test   
Upload Task.cpp inside the Test folder, also make a file Task.cmake

Then add these lines at the bottom line of caravela.ini which is located in ~dune/source/etc/

[Maneuver.Test]   
Enabled                           = Always   
Entity Label                      = Caravela Plan

then save the file.

Now, compile the Task.cpp
and simulate by 
./dune -c caravela -p Simulation
