#!/usr/bin/env python

import sys
import time

import silverpak
import threading

verbose = True
movementReps = 10

# for synchronization
class Initializer:
    WAIT = "wait"
    SUCCESS = "success"
    FAILURE = "failure"
    def __init__(self, n):
        self._successes = 0
        self._failures = 0
        self._n = n
        self._lock = threading.RLock()
        self._check()
    def _check(self):
        if self._successes + self._failures < self._n:
            self.status = self.WAIT
            return
        if self._failures == 0:
            self.status = self.SUCCESS
        else:
            self.status = self.FAILURE
    def success(self):
        with self._lock:
            self._successes += 1
            self._check()
    def failure(self):
        with self._lock:
            self._failures += 1
            self._check()
    def is_success(self):
        while True:
            with self._lock:
                if self.status == self.SUCCESS:
                    return True
                if self.status == self.FAILURE:
                    return False
            time.sleep(0.1)


def main():
    def make_thread(motor):
        def run():
            # connect
            if not motor.findAndConnect():
                print("ERROR: can't find " + motor.id)
                initialization.failure()
                return
            initialization.success()
            if verbose:
                # entire message at once for thread safety
                print(
                    "===== found " + motor.id + " =====\n" +
                    "".join(
                        "  " + attribute + " = " + repr(motor.__dict__[attribute]) + "\n"
                        for attribute in ("portName", "baudRate", "driverAddress")
                    ) +
                    "============================="
                )
            if not initialization.is_success():
                return

            # init motor
            initialized = threading.Event()
            stopped = threading.Event()
            def stoppedHandler(reason):
                if reason == silverpak.StoppedMovingReason.Initialized:
                    initialized.set()
                elif reason == silverpak.StoppedMovingReason.Normal:
                    stopped.set()
            motor.stoppedMovingHandlers.append(stoppedHandler)
            motor.fullInit()
            initialized.wait()

            # move around
            for _ in range(movementReps):
                def moveAndWait(position):
                    stopped.clear()
                    motor.moveToPosition(position)
                    stopped.wait()
                moveAndWait(motor.maxPosition)
                moveAndWait(0)
        return threading.Thread(target=run, name=motor.id + " thread")
    threads = [make_thread(motor) for motor in (motor17, motor23)]
    global initialization
    initialization = Initializer(len(threads))
    for thread in threads:
        thread.start()
    if not initialization.is_success():
        return
    for thread in threads:
        thread.join()


def makePositionUpdated(motor):
    def positionUpdated():
        print(motor.id + ".position: " + str(motor.position()))
    return positionUpdated
def makeStoppedMoving(motor):
    def stoppedMoving(reason):
        message = {
            silverpak.StoppedMovingReason.Normal: "normal",
            silverpak.StoppedMovingReason.Initialized: "initialized",
            silverpak.StoppedMovingReason.InitializationAborted: "aborted",
        }[reason]
        print(motor.id + ".position: " + str(motor.position() + "  stopped: " + message))
    return stoppedMoving
def maybeAttachVerboseHandlers(motor):
    if not verbose:
        return
    motor.positionChangedHandlers.append(makePositionUpdated(motor))
    motor.stoppedMovingHandlers.append(makeStoppedMoving(motor))

def create17():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = 5
    motor.fancy = False
    motor.velocity = 300000
    motor.acceleration = 500
    motor.maxPosition = 242000 * 2
    motor.id = "motor17"
    maybeAttachVerboseHandlers(motor)
    return motor

def create23():
    motor = silverpak.Silverpak()
    motor.baudRate = 9600
    motor.driverAddress = 1
    motor.maxPosition = 10000
    motor.id = "motor23"
    maybeAttachVerboseHandlers(motor)
    return motor


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        if arg in ("-q", "--quiet", "-s", "--silent", "--stfu"):
            verbose = False
        else:
            raise None
    global motor17, motor23
    with create17() as motor17, create23() as motor23:
        main()

