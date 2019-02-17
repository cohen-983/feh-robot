# feh-robot
Serves as a repository for the code written to run team Star Trak's robot.
Code in this repository is intended to be run on the Ohio State University's Proteus controller, maintained by the department of Engineering Education.
## Functions
### Move
Takes an argument of the form (int x, float y).
The integer x defines power percentage, and the float y defines distance to move.
The y argument provided can be of either sign, the x argument should be positive.
### Turn
Takes an argument of the form (bool t, int x, float y).
The boolean t determines whether the turn is a pivot. A value of true pivots the robot, a value of false turns it normally. The integer x defines power percentage, and the float y defines the degrees to turn.
The boolean can accept either true or false as an input, the value of x should be positive, and the value of y can range between positive and negative 360.