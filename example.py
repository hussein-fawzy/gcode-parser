from gcode_parser import GCodeParser, Machine3D

def print_machining_times(file_name):
    #set machine parameters and create machine
    MAX_FEEDRATE = {"X": 300, "Y": 300, "Z": 5}				#mm/s
    ACCELERATIONS = {"X": 350.0, "Y": 350.0, "Z": 100.0}	#mm / s^2
    JERKS = {"X": 10.0, "Y": 10.0, "Z": 0.4}

    machine = Machine3D(MAX_FEEDRATE, ACCELERATIONS, JERKS)

    #create gcode parser
    parser = GCodeParser(file_name, machine)
    parser.parse()

    #print machining times
    times = parser.get_times()

    print("Total Time: {}s".format(round(times[0], 2)))
    print("Acceleration Time: {}s".format(round(times[1], 2)))
    print("Deceleration Time: {}s".format(round(times[2], 2)))

if __name__ == "__main__":
    gcode_file_name = "actions.txt"
    print_machining_times(gcode_file_name)
