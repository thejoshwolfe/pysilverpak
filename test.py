#!/usr/bin/env python3

import sys
import silverpak

def main():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = silverpak.getDriverAddress(1)
    
    if not motor.FindAndConnect():
        sys.exit("no silverpak found")
    
    motor.InitializeMotorSettings()
    motor.InitializeSmoothMotion()
    motor.InitializeCoordinates()
    
    motor.dispose()

if __name__ == "__main__":
    main()
