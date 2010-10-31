#!/usr/bin/env python

import sys
import time
import optparse

import silverpak

done_yet = False

def main(options):
    global motor
    motor = silverpak.Silverpak()
    if options.portName != None:
        motor.portName = options.portName
    if options.baudRate != None:
        motor.baudRate = options.baudRate
    if options.driverAddress != None:
        motor.driverAddress = silverpak.getDriverAddress(options.driverAddress)

    if not motor.findAndConnect():
        sys.exit("no silverpak found")
    try:
        print("===== found a silverpak =====")
        for attribute in ("portName", "baudRate", "driverAddress"):
            print("  " + attribute + ": " + repr(motor.__dict__[attribute]))
        print("=============================")

        motor.positionChangedHandlers.append(positionUpdated)
        motor.stoppedMovingHandlers.append(stoppedMoving)

        if input("send initialization? [Yn]") != "n":
            motor.fullInit()
        while True:
            motor.sendRawCommand(input())
    finally:
        motor.dispose()

def positionUpdated():
    position = motor.getPosition()
    print("position: " + repr(position))

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
    parser = optparse.OptionParser()
    parser.add_option("-p", "--portName", help="such as COM1")
    parser.add_option("-b", "--baudRate", type=int, help="one of 9600, 19200, 38400")
    parser.add_option("-d", "--driverAddress", type=int, help="integer in the range 0-15")
    (options, args) = parser.parse_args()

    main(options)

