
import qwiic_otos
import sys
import time

def runExample():
    print("\nQwiic OTOS Example 1 - Basic Readings\n")

    myOtos = qwiic_otos.QwiicOTOS()

    if myOtos.is_connected() == False:
        print("The device isn't connected to the system. Please check your connection", \
            file=sys.stderr)
        return

    myOtos.begin()

    print("Ensure the OTOS is flat and stationary during calibration!")
    for i in range(5, 0, -1):
        print("Calibrating in %d seconds..." % i)
        time.sleep(1)

    print("Calibrating IMU...")

    myOtos.calibrateImu()

    myOtos.resetTracking()

    while True:
        myPosition = myOtos.getPosition()

        print()
        print("Position:")
        print("X (Inches): {}".format(myPosition.x))
        print("Y (Inches): {}".format(myPosition.y))
        print("Heading (Degrees): {}".format(myPosition.h))

        time.sleep(0.5)


if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("\nEnding Example")
        sys.exit(0)