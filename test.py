#!/usr/bin/env python

import sys
import time

import silverpak

done_yet = False

def main():
    global motor
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = silverpak.getDriverAddress(1)

    if not motor.findAndConnect():
        sys.exit("no silverpak found")
    print("===== found a silverpak =====")
    for attribute in ("portName", "baudRate", "driverAddress"):
        print("  " + attribute + " = " + repr(motor.__dict__[attribute]))
    print("=============================")

    motor.positionChangedHandlers.append(positionUpdated)
    motor.stoppedMovingHandlers.append(stoppedMoving)

    motor.fullInit()

    while not done_yet:
        time.sleep(0.1)

    motor.dispose()

def positionUpdated():
    position = motor.getPosition()
    print("position: " + str(position))

def stoppedMoving(reason):
    message = {
        silverpak.StoppedMovingReason.Normal: "normal",
        silverpak.StoppedMovingReason.Initialized: "initialized",
        silverpak.StoppedMovingReason.InitializationAborted: "aborted",
    }[reason]
    print("stopped: " + message)
    
    if message == silverpak.StoppedMovingReason.Initialized:
        motor.goToPosition(100000)
    elif message == silverpak.StoppedMovingReason.Normal:
        global done_yet
        done_yet = True

if __name__ == "__main__":
    main()
