#!/usr/bin/env python

import sys
import time

import silverpak

done_yet = False

def main():
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

def create17():
    motor = silverpak.Silverpak()
    motor.fancy = False
    motor.velocity = 300000
    motor.acceleration = 500
    motor.baudRate = 9600
    motor.driverAddress = 1
    return motor

def create23():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = 1
    return motor

def positionUpdated():
    print("position: " + str(motor.position()))

def stoppedMoving(reason):
    message = {
        silverpak.StoppedMovingReason.Normal: "normal",
        silverpak.StoppedMovingReason.Initialized: "initialized",
        silverpak.StoppedMovingReason.InitializationAborted: "aborted",
    }[reason]
    print("stopped reason: " + message + ". position: " + str(motor.position()))

    if reason == silverpak.StoppedMovingReason.Initialized:
        newPosition = 5000
        print("going to " + str(newPosition))
        motor.goToPosition(newPosition)
    elif reason == silverpak.StoppedMovingReason.Normal:
        print("done")
        global done_yet
        done_yet = True

if __name__ == "__main__":
    global motor
    args = sys.argv[1:]
    if args[0] == "23":
        motor = create23()
    elif args[0] == "17":
        motor = create17()
    else:
        sys.exit("usage: (17|23)")
    main()
