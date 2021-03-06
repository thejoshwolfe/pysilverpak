#!/usr/bin/env python

import sys
import time
import optparse

import silverpak

def create17():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = 5
    motor.fancy = False
    motor.velocity = 300000
    motor.acceleration = 500
    motor.maxPosition = 242000 * 2
    motor.id = "motor17"
    return motor

def create23():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = 1
    motor.maxPosition = 10000
    motor.id = "motor23"
    return motor

presets = {
    "17": create17,
    "23": create23,
}

def main(options):
    global motor
    if options.preset != None:
        motor = presets[options.preset]()
    else:
        motor = silverpak.Silverpak()
        if options.portName != None:
            motor.portName = options.portName
        if options.baudRate != None:
            motor.baudRate = options.baudRate
        if options.driverAddress != None:
            motor.driverAddress = options.driverAddress

    if not motor.findAndConnect():
        sys.exit("no silverpak found")
    try:
        print("===== found a silverpak =====")
        for attribute in ("portName", "baudRate", "driverAddress"):
            print("  " + attribute + ": " + repr(motor.__dict__[attribute]))
        print("=============================")

        motor.positionChangedHandlers.append(positionUpdated)
        motor.stoppedMovingHandlers.append(stoppedMoving)
        motor.connectionLostHandlers.append(connectionLost)

        if options.raw:
            print("entering raw mode")
            while True:
                motor.sendRawCommand(input(">>> "))
        else:
            fancyConsole()
    except EOFError:
        pass
    finally:
        motor.dispose()

def fancyConsole():
    print("type 'help' for info")
    while True:
        line = input(">>> ")
        if line.strip() == "":
            continue
        call = not line.startswith(" ")
        if line.startswith("-"):
            result = motor.sendRawCommand(line[1:])
            print(repr(result))
            continue
        parts = line.split(None)
        command = parts[0]
        args = parts[1:]
        if command in ("?", "help"):
            print("type methods for the motor and any parameters")
            print("examples:")
            print("    fullInit")
            print("    goInfinite True")
            print("    goToPosition 5000")
            print("prefix the command with a space to print attributes instead of executing methods")
            print("prefix the command with a '-' to signify a raw command")
            print("type 'dir' to list members")
            continue
        if command == "dir":
            print(" ".join(member for member in dir(motor) if not member.startswith("_") and member[0].islower()))
            continue
        try:
            member = getattr(motor, command)
        except AttributeError as e:
            print("ERROR: " + str(e))
            continue
        if not call:
            print(repr(member))
            continue
        actual_args = [eval(arg) for arg in args]
        try:
            result = member(*actual_args)
        except TypeError as e:
            print("ERROR: " + str(e))
            continue
        if result != None:
            print(repr(result))

def positionUpdated():
    position = motor.position()
    print("position: " + repr(position))

def stoppedMoving(reason):
    message = {
        silverpak.StoppedMovingReason.Normal: "normal",
        silverpak.StoppedMovingReason.Initialized: "initialized",
        silverpak.StoppedMovingReason.InitializationAborted: "aborted",
    }[reason]
    print("stopped: " + message)
def connectionLost():
    print("connection was closed")


if __name__ == "__main__":
    parser = optparse.OptionParser()
    parser.add_option("-p", "--portName", help="such as COM1")
    parser.add_option("-b", "--baudRate", type=int, help="one of 9600, 19200, 38400")
    parser.add_option("-d", "--driverAddress", type=int, help="integer in the range 0-15")
    parser.add_option("-r", "--raw", action="store_true", default=False, help="raw communication mode")
    parser.add_option("-t", "--preset", help="see the mapping 'presets' in this module")
    (options, args) = parser.parse_args()

    main(options)

