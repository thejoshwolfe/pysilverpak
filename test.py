
import sys
import silverpak

def main():
    motor = silverpak.SilverpakManager()
    motor.baudRate = 9600
    motor.driverAddress = silverpak.getDriverAddress(1)
    
    if not motor.FindAndConnect():
        sys.exit("no silverpak found")


if __name__ == "__main__":
    main()
