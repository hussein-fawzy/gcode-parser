GCode Parser is used to calculate the time taken for a machine to complete a GCode file.
The implementation takes into consideration max allowable feedrate, max allowable acceleration and max allowable jerk.

The calculated parameters are:
- Total Time: The total time taken by the machine to complete the GCode file from start to finish 
- Acceleration Time: The total time taken by all acceleration moves
- Deceleration Time: The total time taken by all deceleration moves

Using the parser:
- Place the gcode file in the same directory as the script files
1- Open example.py
2- Change MAX_FEEDRATE, ACCELERATIONS and JERKS to match the machine parameters
3- Change gcode_file_name to be the file name, including extension, of the gcode file
4- Run example.py

Scripts:
executioncontrol.py: Used to debug and evaluate the execution time of the GCode parser itself (ex. How much time did a certain method in the parser take to execute?)
gcodeparser.py: Main GCode parser
geometry: Required mathematical functions
