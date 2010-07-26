#!/usr/bin/env python3

import sys
import silverpak

def main():
    global motor
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = silverpak.getDriverAddress(1)
    
    if not motor.findAndConnect():
        sys.exit("no silverpak found")
    
    motor.positionChangedHandlers.append(positionUpdated)
    motor.stoppedMovingHandlers.append(stoppedMoving)
    
    motor.fullInit()
    
    
    motor.dispose()

def positionUpdated():
    position = motor.getPosition()
    print("position: " + str(position))

def stoppedMoving(reason):
    message = {
        silverpak.StoppedMovingReason.Normal: "normal",
        silverpak.StoppedMovingReason.Initialized: "initialized",
        silverpak.StoppedMovingReason.InitializationAborted: "aborted",
    }[StoppedMovingReason]
    print("stopped: " + message)
    
    if message == silverpak.StoppedMovingReason.Initialized:
        motor.goToPosition(100000)

if __name__ == "__main__":
    main()
