# Flight simulator

A flight simulator written in C running using  OpenGL and GLUT.

The object of the game is to fly an airplane through rings, sometimes stationary, sometimes moving. Points are awarded for flying through a ring, and deducted for missing a ring. Lives are lost for colliding with a ring.

![Gameplay image](https://github.com/joshtyler/Flight-Simulator/raw/master/Docs/Images/Picture1.png "Main menu, demonstrating autopilot")

##Control
The game is controlled with either a keyboard alone, keyboard and mouse or an Xbox 360 controller. The Xbox 360 controller provides the best overall experience, including vibration feedback.

Keyboard and mouse input is provided using GLUT, and Xbox controller input uses a proprietary Microsoft library.

Different difficultly levels are implemented.

## Technical details
The source code is provided as a Visual Studio project. The code is written in fairly standard C, so should compile in other environments, provided that the relevant libraries are linked in correctly for your compiler,and the Xbox controller related code is removed.

The code is quite messy,and non ideal. It was my first semi-serious OpenGL project, so it was mostly a learning experience, and many features were added in a hurry at the last minute (this project was for a university assignment). If I have time I hope to tidy it up in the future.

In addition, there is an issue where the speed of the game play varies wildly with the speed of your computer. This should be fairly easy to fix, I think it is due to the fact that the control loop occurs only when the OpenGL/GLUT has time.

The engine is fairly feature rich given the scope of the project, and advanced features, including loading maps from text files and and an autopilot are included.

All code, including the game engine, GUI, and autopilot  is written from scratch, with the exception of two libraries, which are credited where appropriate.

##Documentation
A pdf is provided in docs giving an overview of the project, and how the code works. This includes slightly bizarre details in order to meet the university criteria for it.

##License
This project is released under the MIT license, see LICENSE.txt.

You may do whatever you please with this code, so long as attribution is given to me.