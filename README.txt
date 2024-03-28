Code Explanation:

- The code uses Standard caliberation functions to setup lineSensors and Proximity Sensors.
- To define a standard unit distance, the code considers the distance covered by the robot moving at 100 speed for 500 milliseconds as measured by the encoders.
- The code uses a pre-defined set of directions to navigate the maze while also mapping the maze in an array 0s and 1s with 1 representing path and 0 everything else.
- As the robot moves through the maze, the array is updated with 1s in place of 0s.
- the code detects for objects in rooms and rcords the coordinate.
- The code uses manhatten distance to measure distance to objects from current position, finding shortest distance.
- the code navigates thorugh maze to object with the shortest distance, based on the array of 0s and 1s.

Building Program.

 - Open .ino file in Arduino IDE
 - Connect Zumo 32U4 robot.
 - Upload code to robot.


Scenario 1,2 & 3

- Start the robot from bottom roght of the maze. This will be the robot's base.
- The robot will go around the whole maze and find all objects in the maze.
- The robot will return to initial position
- The robot will navigate to the objects in the maze it found.
- The robot will return back to base after going to each object.

References:

1. Zumo32U4 library: Zumo32U4 library. (n.d.). Pololu.github.io. https://pololu.github.io/zumo-32u4-arduino-library/

‌2. Singh, V. (2022). All about Manhattan Distance - Shiksha Online. [online] Shiksha.com. Available at: https://www.shiksha.com/online-courses/articles/all-about-manhattan-distance/.

3.Introduction to A*. (n.d.). Theory.stanford.edu. https://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html

‌4. A* Pathfinding in the Arduino. (2016, November 13). Arduino Forum. https://forum.arduino.cc/t/a-pathfinding-in-the-arduino/418541

‌5. zumo-32u4-arduino-library/examples/SumoProximitySensors/SumoProximitySensors.ino at master · pololu/zumo-32u4-arduino-library. (n.d.). GitHub. Retrieved March 27, 2024, from https://github.com/pololu/zumo-32u4-arduino-library/blob/master/examples/SumoProximitySensors/SumoProximitySensors.ino

‌
