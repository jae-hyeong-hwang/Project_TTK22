# Project work of TTK22 Software toolchain for networked vehicle systems
TTK22 is a project course of NTNU, during autumn semester.
The aim of this course is to learn about [LSTS Toolchain](https://www.lsts.pt/toolchain).  
Briefly speaking, LSTS Toolchain is a software which enables to connect multi-vehicles regardless of its type.  
project work of TTK22, make asv(autonomous surface vehicle) Caravela search certain area.  

![Screenshot from 2021-03-26 19-02-08](https://user-images.githubusercontent.com/71170784/112674682-7f63fe00-8e66-11eb-8532-ee00ddc15403.png)

You should have installed LSTS Toolchain and have to know basic commands and how system works.
Please have a look at the [github page of LSTS Toolchain](https://github.com/LSTS) or the [official website](https://www.lsts.pt/toolchain)

if you are running in your own system, make a folder "Test" inside Maneuver.
so the directory will be   
`~ dune/source/src/Maneuver/Test` 
Upload Task.cpp inside the Test folder, also make a file `Task.cmake`

Then add these lines at the bottom line of caravela.ini which is located in `~dune/source/etc/`

>[Maneuver.Test]   
Enabled                           = Always   
Entity Label                      = Caravela Plan

then save the file.

Now, compile the Task.cpp
and simulate by 
`./dune -c caravela -p Simulation`
