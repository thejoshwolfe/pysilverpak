"""
module for controlling a Silverpak stepper motor over a serial port in Windows
with 32-bit python 3.

this module requires pyserial for python 3.
pyserial-py3k-2.5-rc1.win32.exe works.

supported devices:
 - Silverpak 23CE firmware version 'Lin Engineering V6.95 10-17-07    '
   - http://www.linengineering.com/LinE/contents/stepmotors/SilverPak_23CE.aspx
   - http://www.linengineering.com/LinE/contents/stepmotors/pdf/Silverpak23C-R356Commands.pdf
 - Silverpak 17C firmware version 'Lin Engineering 9/22/2006 v5.71'
   - http://www.linengineering.com/LinE/contents/stepmotors/SilverPak_17CE.aspx
   - http://www.linengineering.com/LinE/contents/stepmotors/pdf/Silverpak17C-256uStepping.pdf
"""

import sys
import time, threading

import serial


__all__ = []

__all__.append("make_thread")
def make_thread(target, name, args=[]):
    """
    overwrite this method with your own if you want
    """
    return threading.Thread(target=target, name=name, args=args)

__all__.append("Silverpak")
class Silverpak:
    """Provides an interface to a Lin Engineering Silverpak stepper motor"""

    DefaultAcceleration = 50
    DefaultBaudRate = -1
    DefaultDriverAddress = None
    DefaultEncoderRatio = 10266
    DefaultHoldingCurrent = 5
    DefaultHomePolarity = 0
    DefaultMaxPosition = 500000
    DefaultMotorPolarity = 1
    DefaultPortname = ""
    DefaultPositionCorrectionRetries = 5
    DefaultPositionCorrectionTolerance = 5
    DefaultPositionUpdaterInterval = 200
    DefaultRunningCurrent = 50
    DefaultVelocity = 30000

    TolerableCommunicationFailureCount = 5

    # configurable
    errorCallback = None

    def isActive(self):
        """Returns a value indicating whether this Silverpak is actively connected to a Silverpak"""
        with self._motor_lock:
            return self._motorState_motor != MotorStates.Disconnected
    def isReady(self):
        """whether the motor is ready to accept a command. i.e. connected and stopped."""
        if self._fake:
            return not self._fake_moving
        with self._motor_lock:
            return self._motorState_motor == MotorStates.Stopped
    def position(self):
        if self._fake:
            return self._fake_position
        return self._position

    def __init__(self):
        self._connectionManager_motor = SilverpakConnectionManager()
        self.acceleration = self.DefaultAcceleration
        self.baudRate = self.DefaultBaudRate
        self.driverAddress = self.DefaultDriverAddress
        self.encoderRatio = self.DefaultEncoderRatio
        self.holdingCurrent = self.DefaultHoldingCurrent
        self.homePolarity = self.DefaultHomePolarity
        self.maxPosition = self.DefaultMaxPosition
        self.motorPolarity = self.DefaultMotorPolarity
        self.portName = self.DefaultPortname
        self.positionCorrectionRetries = self.DefaultPositionCorrectionRetries
        self.positionCorrectionTolerance = self.DefaultPositionCorrectionTolerance
        self.positionUpdaterInterval = self.DefaultPositionUpdaterInterval
        self.runningCurrent = self.DefaultRunningCurrent
        self.velocity = self.DefaultVelocity
        self._position = 0
        self.enableLimitSwitches = True
        self.enablePositionCorrection = True
        self._fake = False

        # Fields in the lock group: motor
        # Lock object for the lock group: motor.
        self._motor_lock = threading.RLock()
        # Connection manager component. Part of the lock group: motor.
        self._connectionManager_motor = SilverpakConnectionManager()
        # The present state of the motor. Part of the lock group: motor.
        self._motorState_motor = MotorStates.Disconnected

        # Fields in the lock group: posUpd
        # Lock object for the lock group: posUpd.
        self._posUpd_lock = threading.RLock()
        # Used to cancel the position updater thread. Part of the lock group: posUpd.
        self._keepPositionUpdaterRunning_posUpd = False
        # Thread that periodically gets the position of the motor. Part of the lock group: posUpd.
        self._positionUpdaterThread_posUpd = None

        # called when the connection to the Silverpak is lost.
        self.connectionLostHandlers = []
        # called when the motor stops moving.
        self.stoppedMovingHandlers = []
        # called when the motor's position changes. Read the Position property to get the position.
        self.positionChangedHandlers = []

        self._homeCalibrationSteps = 0
        # track the number of times we didn't receive a valid response
        self._failCount = 0

    def setFake(self, speed=1234):
        self._fake = True
        self._fake_position = 0
        self._fake_destination = 0
        self._fake_stopTheMotor = False
        self._fake_moving = False
        self._fake_speed = speed

    # Public methods
    def connect(self):
        """
        Attempts to connect to a Silverpak. 
        The portName, baudRate, and driverAddress properties must be set or an ValueError will be raised. 
        To auto-detect these properties, see findAndConnect(). 
        The isActive property must return False when calling this method or an InvalidSilverpakOperationException will be raised.
        """
        if self._fake:
            return True
        with self._motor_lock:
            # Validate state and connection properties
            if self._motorState_motor != MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is already active.")
            if self.portName == self.DefaultPortname: raise ValueError("portName property must be set before calling this method. See also findAndConnect().")
            if self.baudRate == self.DefaultBaudRate: raise ValueError("baudRate property must be set before calling this method. See also findAndConnect().")
            if self.driverAddress == self.DefaultDriverAddress: raise ValueError("driverAddress property must be set before calling this method. See also findAndConnect().")

            # Initialize connection manager's properties
            self._connectionManager_motor.portName = self.portName
            self._connectionManager_motor.baudRate = self.baudRate
            self._connectionManager_motor.driverAddress = self.driverAddress
            # Attempt to connect
            if self._connectionManager_motor.connect():
                # Connection succeeded
                self._motorState_motor = MotorStates.Stopped
                return True
            else:
                # Connection failed
                return False

    def findAndConnect(self):
        """
        Attempts to find and connect to a Silverpak. 
        if any of the properties PortName, BaudRate, and DriverAddress are set to their defaults, all of their possible values will be searched. 
        After a successful connection, these properties will be set to their discovered values. 
        The isActive property must return False when calling this method or an InvalidSilverpakOperationException will be raised.
        """
        if self._fake:
            return True
        with self._motor_lock:
            # Validate state
            if self._motorState_motor != MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is already active")

            # Get information for all COM ports being searched
            portInfos = SearchComPorts(self.portName, self.baudRate, self.driverAddress)
            # Search the list of information for an available Silverpak
            for portInfo in portInfos:
                if portInfo.portStatus != PortStatuses.AvailableSilverpak:
                    continue
                # Listed available Silverpak found
                # Initialize connection manager's properties
                self._connectionManager_motor.portName = portInfo.portName
                self._connectionManager_motor.baudRate = portInfo.baudRate
                self._connectionManager_motor.driverAddress = portInfo.driverAddress
                # Attempt to connect
                if not self._connectionManager_motor.connect():
                    # this should only happen in the rare event that the Silverpak was disconnected between the call to SearchComPorts and now
                    continue
                # Connection succeeded
                # Save connection properties
                self.portName = portInfo.portName
                self.baudRate = portInfo.baudRate
                self.driverAddress = portInfo.driverAddress
                self._motorState_motor = MotorStates.Stopped
                return True
            # End of list was reached and no available Silverpak was found
            return False

    def fullInit(self):
        """calls the initalization methods in order"""
        self.initializeMotorSettings()
        self.initializeSmoothMotion()
        self.initializeCoordinates()

    def sendRawCommand(self, command):
        """such as 'A10000'"""
        return self._connectionManager_motor.writeAndGetResponse(GenerateMessage(self.driverAddress, command), 4.0)
    def initializeMotorSettings(self):
        """
        Initialization Step 1. 
        This method will set the motor settings as specified by this Silverpak's properties.
        This method does not cause the motor to move.
        The next step is initializeSmoothMotion().
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        if self._fake:
            return
        with self._motor_lock:
            # Validate state
            self._checkStopped()
            # Send settings-initialization command
            self._connectionManager_motor.write(GenerateMessage(self.driverAddress, self._generateFullInitCommandList()), 5.0)
            # Update state
            self._motorState_motor = MotorStates.Stopped

    def _checkStopped(self):
        """must be in Stopped state, meaning ready to go."""
        self._checkConnected()
        if self._motorState_motor != MotorStates.Stopped:
            raise InvalidSilverpakOperationException("operation cannot be performed while moving.")
    def _checkConnected(self):
        if self._motorState_motor == MotorStates.Disconnected:
            raise InvalidSilverpakOperationException("Connection is not active.")

    def resendMotorSettings(self):
        """
        Call this method if any changes to the motor settings properties need to be applied.
        This method does not cause the motor to move.
        Calling this method will raise an InvalidSilverpakOperationException if the isActive property returns False or if the motor is moving.
        """
        if self._fake:
            return
        with self._motor_lock:
            # Validate state
            self._checkStopped()
            # Send settings command
            self._connectionManager_motor.write(GenerateMessage(self.driverAddress, self._generateResendInitCommandList()), 4.0)

    def initializeSmoothMotion(self):
        """
        Initialization Step 2. 
        This method will send a small motion command five times to bypass any initialization quirks that some motors are prone to exhibit.
        This method causes the motor to move 5 microsteps in the positive direction and causes the motor to briefly produce a rapid tapping sound.
        The next step is initializeCoordinates().
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        if self._fake:
            return
        with self._motor_lock:
            # Validate state
            self._checkStopped()

            # Send a small motion command 5 times
            smoothMotionInitMsg = GenerateMessage(self.driverAddress, Commands.GoPositive + "40")
            for _ in range(5):
                self._connectionManager_motor.write(smoothMotionInitMsg, 3.0)
            # Update state
            self._motorState_motor = MotorStates.Stopped
    def initializeCoordinates(self):
        """
        Initialization Step 3. 
        This method will send the motor looking for its upper limit switch so it can zero its coordinate system.
        This method causes the motor to move in the negative direction until it trips the upper limit switch.
        The next initialization step is to wait for the CoordinatesInitialized event or to wait for the IsReady property to return True.
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        if self._fake:
            def waitAndNotify():
                # raise completed event after 1 second
                time.sleep(1)
                self._onPositionChanged()
                self._onCoordinatesInitialized()
            make_thread(waitAndNotify, "init notifier").start()
            return
        with self._motor_lock:
            # Validate state
            self._checkStopped()
            if self.enableLimitSwitches:
                self._moveToZero()
            else:
                # don't go anywhere. assumer here is 0.
                cmd = Commands.SetPosition + "0"
                message = GenerateMessage(self.driverAddress, cmd)
                # get a response to clear the read buffer
                self._connectionManager_motor.writeAndGetResponse(message, 1.0)
        if not self.enableLimitSwitches:
            # no stopping to trigger this, so call it now.
            self._onCoordinatesInitialized()
        # Now that the motor is moving, begin listening for position changes
        self._startPositionUpdater()

    def _moveToZero(self):
        # move to zero in preparation for home calibration
        cmd = Commands.SetPosition + str(int(self.maxPosition * (self.encoderRatio / 1000.0)))
        if self.enablePositionCorrection:
            cmd += Commands.SetEncoderRatio + str(self.encoderRatio)
        cmd += Commands.GoAbsolute + "0"
        message = GenerateMessage(self.driverAddress, cmd)
        self._connectionManager_motor.write(message, 2.0)

        # Update state
        self._motorState_motor = MotorStates.InitializingCoordinates_moveToZero

    def goInfinite(self, positive):
        """
        Sends the motor either to position 0 or to the position specified by the MaxPosition property.
        Calling this method before the motor has been fully initialized will raise an InvalidSilverpakOperationException.
        """
        self.goToPosition({True: self.maxPosition, False: 0}[positive])

    def stop(self):
        """
        Stops the motor.
        Calling this method when the isActive property returns False will raise an InvalidSilverpakOperationException.
        """
        if self._fake:
            self._fake_stopTheMotor = True
            return
        with self._motor_lock:
            # Validate state
            self_checkConnected()

            # Send stop command
            stopMessage = GenerateMessage(self.driverAddress, Commands.TerminateCommand)
            self._connectionManager_motor.write(stopMessage, 1.0)
            # Update state if applicable
            if self._motorState_motor in (
                    MotorStates.InitializingCoordinates_moveToZero,
                    MotorStates.InitializingCoordinates_calibrateHome,
                ):
                self._motorState_motor = MotorStates.AbortingCoordinateInitialization

    def goToPosition(self, position):
        """
        Sends the motor to the passed position.
        Calling this method before the motor has been fully initialized will raise an InvalidSilverpakOperationException.
        """
        if self._fake:
            self._fake_destination = position
            def animate():
                def sign(number):
                    if number > 0:
                        return 1
                    if number < 0:
                        return -1
                    return 0
                direction = sign(self._fake_destination - self._fake_position)
                if direction == 0:
                    return
                velocity = direction * self._fake_speed
                almost_there_position = self._fake_destination - velocity

                # here we go
                self._fake_stopTheMotor = False
                self._fake_moving = True
                try:
                    while direction * self._fake_position < direction * almost_there_position:
                        if self._fake_stopTheMotor:
                            return
                        self._fake_position += velocity
                        self._onPositionChanged()
                        time.sleep(0.05)
                    # close enough. snap to destination
                    self._fake_position = self._fake_destination
                    self._onPositionChanged()
                    time.sleep(0.05)
                finally:
                    self._fake_moving = False
                    self._onStoppedMoving()
            # spawn a thread for animation
            make_thread(animate, "position animator").start()
            return
        # real method
        with self._motor_lock:
            # Validate state
            if self._motorState_motor not in (MotorStates.Stopped, MotorStates.Moving):
                raise InvalidSilverpakOperationException("Motor is not fully initialized")
            # Send absolute motion command
            self._connectionManager_motor.write(GenerateMessage(self.driverAddress, Commands.GoAbsolute + str(position)), 1.0)
            # Update state
            self._motorState_motor = MotorStates.Moving

    def disconnect(self):
        """
        Terminates the connection to the Silverpak and closes the COM port.
        Calling this method will raise an InvalidSilverpakOperationException if the isActive property returns False or if the motor is moving.
        """
        if self._fake:
            return
        with self._motor_lock:
            # Validate state
            self._checkConnected()
            if self._motorState_motor == MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is not active.")
            if self._motorState_motor in MotorStates.moving:
                raise InvalidSilverpakOperationException("Disconnecting while the motor is moving is not allowed.")

            # disconnect
            self._connectionManager_motor.disconnect()
            # Update state
            self._motorState_motor = MotorStates.Disconnected

    def _notify(self, handlers, *args):
        for handler in handlers[:]:
            handler(*args)
    def _onConnectionLost(self):
        self._notify(self.connectionLostHandlers)
    def _onCoordinatesInitializationAborted(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.InitializationAborted)
    def _onCoordinatesInitialized(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.Initialized)
    def _onPositionChanged(self):
        self._notify(self.positionChangedHandlers)
    def _onStoppedMoving(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.Normal)

    def __enter__(self):
        return self
    def __exit__(self, *_):
        self.dispose()
    def dispose(self):
        """cleans up and shuts down. it is always safe to call this method"""
        with self._motor_lock:
            if self._isMoving_motor():
                self._connectionManager_motor.write(GenerateMessage(self.driverAddress, Commands.TerminateCommand), 1.0)
            self._stopPositionUpdater()
            self._connectionManager_motor.disconnect()
            self._motorState_motor = MotorStates.Disconnected
    def _isMoving_motor(self):
        return self._motorState_motor not in (MotorStates.Disconnected, MotorStates.Stopped)
    def __del__(self):
        """in case users don't dispose this object properly"""
        self._keepPositionUpdaterRunning_posUpd = False

    # Makes sure the position updater thread is running.
    def _startPositionUpdater(self):
        with self._posUpd_lock:
            self._keepPositionUpdaterRunning_posUpd = True # make sure the position updater thread doesn't cancel
            if self._positionUpdaterThread_posUpd == None: # only activate it when it's not active
                self._positionUpdaterThread_posUpd = make_thread(self._positionUpdater_run, None)
                self._positionUpdaterThread_posUpd.daemon = True
                self._positionUpdaterThread_posUpd.start()

    def _stopPositionUpdater(self):
        """Stops the position updater thread and makes sure it dies."""
        with self._posUpd_lock:
            if self._positionUpdaterThread_posUpd == None:
                return
            if threading.current_thread() ==  self._positionUpdaterThread_posUpd:
                # the position updater thread cannot stop itself; a thread can never see itself die.
                # stop the position updater thread on a seperate thread.
                make_thread(self._stopPositionUpdater_not_positionUpdaterThread, None).start()
            else:
                self._stopPositionUpdater_not_positionUpdaterThread()
    def _stopPositionUpdater_not_positionUpdaterThread(self):
        """Stops the position updater thread and makes sure it dies. This method cannot be called on the position updater thread."""
        try:
            with self._posUpd_lock:
                # cancel the position updater
                self._keepPositionUpdaterRunning_posUpd = False
                # make sure the position updater thread dies before releasing the lock
                self._positionUpdaterThread_posUpd.join(1.0)
                self._positionUpdaterThread_posUpd = None
        except Exception as ex:
            Silverpak._invokeErrorCallback(ex)

    def _positionUpdater_run(self):
        """Method that the position getter thread runs."""
        try:
            # check for cancelation
            while self._keepPositionUpdaterRunning_posUpd:
                # Keep time according to Environment.TickCount
                nextIterationTime = time.time() + self.positionUpdaterInterval / 1000.0
                # Update postion
                self._updatePosition()
                # Wait for the next iteration time
                time.sleep(max(0.0, nextIterationTime - time.time()))
        except Exception as ex:
            Silverpak._invokeErrorCallback(ex)

    def _updatePosition(self):
        """Updates the Position property by querying the position of the motor."""
        # store a function to call after the lock has been released
        postLockAction = None
        try:
            with self._motor_lock:
                getPositionMessage = GenerateMessage(self.driverAddress, Commands.QueryMotorPosition)
                newPosition = None
                response = None
                try:
                    response = self._connectionManager_motor.writeAndGetResponse(getPositionMessage, 1.0)
                except InvalidSilverpakOperationException:
                    # the Silverpak's been disconnected
                    # shut down updater thread
                    postLockAction = self._stopPositionUpdater
                    return
                # Serial Port is still active
                if response != None:
                    if response.endswith("\0"):
                        # the Silverpak17 v5.71 puts a \0 at the end of positions
                        response = response[:-1]
                    try:
                        newPosition = int(response)
                        # Got a valid response
                    except ValueError:
                        pass
                if self._position == newPosition:
                    # motor stopped moving
                    debug("motor stopped moving")
                    self._failCount = 0
                    if self._motorState_motor == MotorStates.InitializingCoordinates_moveToZero:
                        # wait! sometimes the motor will stop at 5000000 and lie about being at the top (stupid old firmware)
                        if abs(self._position - 5000000) < 100:
                            # try again
                            debug("move to 0 stopped at 5000000. try again.")
                            self._moveToZero()
                        else:
                            self._motorState_motor = MotorStates.InitializingCoordinates_calibrateHome
                            # Send the homing message
                            debug("sending the homing message")
                            initCoordMessage = GenerateMessage(self.driverAddress, Commands.GoHome + str(int(self.maxPosition * (self.encoderRatio / 1000.0))))
                            self._connectionManager_motor.write(initCoordMessage, 1.0)
                            self._homeCalibrationSteps = 0
                    elif self._motorState_motor == MotorStates.InitializingCoordinates_calibrateHome:
                        debug("calibrate home is complete.")
                        self._motorState_motor = MotorStates.Stopped
                        postLockAction = self._onCoordinatesInitialized
                    elif self._motorState_motor == MotorStates.AbortingCoordinateInitialization:
                        debug("aborting coordinate initialization complete.")
                        self._motorState_motor = MotorStates.Stopped
                        postLockAction = self._onCoordinatesInitializationAborted
                    elif self._motorState_motor == MotorStates.Moving:
                        debug("normal motion complete.")
                        self._motorState_motor = MotorStates.Stopped
                        postLockAction = self._onStoppedMoving
                    elif self._motorState_motor == MotorStates.Stopped:
                        # sitting stil
                        pass
                    else:
                        warning("stopped in an unexpected state: " + repr(self._motorState_motor))
                elif newPosition != None:
                    debug("motor changed position.")
                    self._failCount = 0
                    self._position = newPosition
                    postLockAction = self._onPositionChanged
                    # make sure the home calibration isn't sneaking away
                    if self._motorState_motor == MotorStates.InitializingCoordinates_calibrateHome:
                        self._homeCalibrationSteps += 1
                        if self._homeCalibrationSteps > 5:
                            # Calling shenanigans on initialization
                            stopMessage = GenerateMessage(self.driverAddress, Commands.TerminateCommand)
                            # stop the motor damnit
                            debug("stopping due to shenanigans.")
                            for _ in range(3):
                                self._connectionManager_motor.write(stopMessage, 1.0)
                            # crash
                            shenanigans = "Motor shenanigans detected! This is a quirk resulting from using outdated motor firmware.\nPlease restart the program."
                            error(shenanigans)
                            sys.exit(shenanigans)
                else:
                    # failed to get a valid position
                    self._failCount += 1
                    debug("bad position specified: " + repr(response) + ". times in a row: " + str(self._failCount))
                    if self._failCount >= self.TolerableCommunicationFailureCount:
                        # failed too many times in a row. Silverpak must no longer be available.
                        self._failCount = 0
                        # disconnect
                        self._motorState_motor = MotorStates.Disconnected
                        self._connectionManager_motor.disconnect()
                        # raise connection lost event
                        postLockAction = self._onConnectionLost
        finally:
            # run action if any
            if postLockAction != None:
                postLockAction()

    def _generateFullInitCommandList(self):
        """Produces a command list to initialize the motor from scratch."""
        initMotorSettingsProgramHeader = Commands.SetPosition + "0"
        mode = 0
        if self.enableLimitSwitches:
            mode |= 2
        if self.enablePositionCorrection:
            mode |= 8
        initMotorSettingsProgramFooter = Commands.SetMode + str(mode)
        return initMotorSettingsProgramHeader + self._generateResendInitCommandList() + initMotorSettingsProgramFooter

    def _generateResendInitCommandList(self):
        """Produces a command list to set the adjustable motor settings."""
        settings = []
        if self.holdingCurrent != None:
            settings.append(Commands.SetHoldingCurrent + str(self.holdingCurrent))
        settings.append(Commands.SetRunningCurrent + str(self.runningCurrent))
        settings.append(Commands.SetMotorPolarity + str(self.motorPolarity))
        if self.enableLimitSwitches:
            settings.append(Commands.SetHomePolarity + str(self.homePolarity))
        if self.enablePositionCorrection:
            settings.append(Commands.SetPositionCorrectionTolerance + str(self.positionCorrectionTolerance))
            settings.append(Commands.SetPositionCorrectionRetries + str(self.positionCorrectionRetries))
            settings.append(Commands.SetEncoderRatio + "1000")
        settings.append(Commands.SetVelocity + str(self.velocity))
        settings.append(Commands.SetAcceleration + str(self.acceleration))
        if self.enablePositionCorrection:
            settings.append(Commands.SetEncoderRatio + str(self.encoderRatio))
        return "".join(settings)

    @staticmethod
    def _invokeErrorCallback(ex):
        """Invokes the ErrorCalback delegate if it has been set. Otherwise, re-throws the exception."""
        if Silverpak.errorCallback != None:
            Silverpak.errorCallback(ex)
        else:
            raise ex


def printTraceback(ex):
    import traceback
    traceback.print_exception(type(ex), ex, ex.__traceback__)


__all__.append("InvalidSilverpakOperationException")
class InvalidSilverpakOperationException(Exception):
    """The exception that is thrown when a method call in namespace Silverpak is invalid for the object's current state."""


class PortInformation:
    """Represents a collection of data for reporting the status of a COM port"""
    def __init__(self, portName=None, baudRate=0, portStatus=None, driverAddress=None):
        self.portName = portName
        self.baudRate = baudRate
        self.portStatus = portStatus
        self.driverAddress = driverAddress
    def __repr__(self):
        return "PortInformation({0})".format(", ".join("{0}={1}".format(*pair) for pair in self.__dict__.items()))


allDriverAddresses = list(range(0x10))
def getDriverAddress(index):
    """0x0 <= index <= 0xf"""
    return "@123456789:;<=>?"[index]


class PortStatuses:
    # Indicates that there is an active, available Silverpak on this COM port
    AvailableSilverpak = "[available]"
    # Indicates that this COM port does not have an active Silverpak
    Empty = "[empty]"
    # Indicates that this COM port could not be read from or written to
    Invalid = "[invalid]"
    # Indicates that this COM port is already open by another resource
    Busy = "[busy]"


__all__.append("StoppedMovingReason")
class StoppedMovingReason:
    """Represents the reason that the motor stopped moving."""
    # The motor stopped after a goInfinite() or goToPosition() command.
    Normal = "[normal]"
    # The initializeCoordinates() command has completed without being interrupted.
    Initialized = "[init]"
    # The initializeCoordinates() command is aborted by calling the stop() method.
    InitializationAborted = "[init_abort]"


# Friend classes

class SilverpakConnectionManager:
    """Manages the connection to a Silverpak through a serial port."""

    # The delay factor for a safe query.
    SafeQueryDelayFactor = 3.0
    # The minimum amount of time in seconds to wait for the Silverpak to respond to a command.
    PortDelayUnit = 0.05

    # Public properties

    # Public constructors
    def __init__(self):
        self.portName = None
        self.baudRate = 0
        self.driverAddress = None

        # Lock object for the serial port
        self._srlPort_lock = threading.RLock()
        # The serial port object used to communicate with a Silverpak.
        self._serialPortInterface_srlPort = makeSerialPort()

        self._nextReadWriteTime = time.time()
    # Public methods
    def connect(self):
        """
        Attempts to connect to a Silverpak using the portName, baudRate, and driverAddress properties. 
        Returns True if successful.
        Throws an InvalidSilverpakOperationException if already connected.
        """
        with self._srlPort_lock:
            # Validate SerialPort state
            if self._serialPortInterface_srlPort.isOpen(): raise InvalidSilverpakOperationException("Already connected.")
            try: # except all expected exceptions
                # apply serial port settings
                self._serialPortInterface_srlPort.port = self.portName
                self._serialPortInterface_srlPort.baudrate = self.baudRate
                # Attempt to connect
                self._serialPortInterface_srlPort.open()
                # Check for a Silverpak
                response = self.writeAndGetResponse_srlPort(GenerateMessage(self.driverAddress, SafeQueryCommandStr), self.SafeQueryDelayFactor)
                if response != None:
                    return True
            except serial.serialutil.SerialException:
                pass
            # Failed to connect. Make sure the SerialPort is closed
            self.closeSerialPort_srlPort()
            return False

    def disconnect(self):
        """Makes sure there is no active connection to a Silverpak."""
        with self._srlPort_lock:
            self.closeSerialPort_srlPort()

    def write(self, completeMessage, delayFactor):
        """
        Writes the passed complete message to the Silverpak.
        Throws an InvalidSilverpakOperationException if not connected.
        <param name="completeMessage">Recommended use GenerateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        with self._srlPort_lock:
            # Validate state
            if not self._serialPortInterface_srlPort.isOpen(): raise InvalidSilverpakOperationException()
            # write message
            self.write_srlPort(completeMessage, delayFactor)

    def writeAndGetResponse(self, completeMessage, delayFactor):
        """
        Writes the passed message to and returns the body of the response from the Silverpak.
        if no response was received, returns null.
        Throws an InvalidSilverpakOperationException if not connected.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        with self._srlPort_lock:
            # Validate state
            if not self._serialPortInterface_srlPort.isOpen(): raise InvalidSilverpakOperationException()
            # write messag and get response
            return self.writeAndGetResponse_srlPort(completeMessage, delayFactor)

    def closeSerialPort_srlPort(self):
        if not self._serialPortInterface_srlPort.isOpen():
            return
        try:
            # Close the serial port.
            self._serialPortInterface_srlPort.close()
        except:
            # Ignore any exceptions that occure while closing.
            pass


    def writeAndGetResponse_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to and returns the body of the response from the Silverpak.
        if no response was received, returns null.
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        # Clear the read buffer.
        garbage = self.safeReadExisting_srlPort(0.0)
        communication("garbage: " + repr(garbage))
        # Write the message.
        self.safeWrite_srlPort(completeMessage, delayFactor)
        # accumulates chunks of RX data (it's O(n^2), but w/e)
        totalRx = ""
        # Read the response from the Silverpak in chunks until the accumulated message is complete.
        responseContent = None
        while True:
            # Read a chunk.
            rxStr = str(self.safeReadExisting_srlPort(1.0), "mbcs")
            if rxStr == "":
                # nothing more to read
                return responseContent
            # Append chunk to accumulated RX data.
            totalRx += rxStr
            # check to see if the accumulated RX data is complete
            if not IsRxDataComplete(totalRx):
                continue # keep accumulating
            # Trim the RX data
            trimResponse = TrimRxData(totalRx)
            communication("trimmed rx data: " + repr(trimResponse))
            # use the last valid response as return message
            responseContent = trimResponse[1:]
            totalRx = ""

    def write_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to the Silverpak.
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        self.safeWrite_srlPort(completeMessage, delayFactor)

    def safeReadExisting_srlPort(self, delayFactor):
        """
        Reads the existing data on the read buffer from the Silverpak after calling waitForSafeReadWrite_srlPort.
        In the event of an unexcepted exception from SerialPort.ReadExisting(), returns null.
        Part of the lock group: srlPort.
        <param name="delayFactor">How long to wait after reading from the Silverpak,
        expressed as a multiple of PortDelatUnit, typically 1.0.</param>
        """
        # wait for safe read/write
        self.waitForSafeReadWrite_srlPort(delayFactor)
        try:
            return self._serialPortInterface_srlPort.read()
        except:
            # except any undocumented exceptions from SerialPort.ReadExisting()
            return None

    def safeWrite_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to the Silverpak after calling waitForSafeReadWrite_srlPort.
        Catches all exceptions from SerialPort.Write().
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        # wait for safe read/write
        self.waitForSafeReadWrite_srlPort(delayFactor)
        try:
            communication("write: " + repr(completeMessage))
            self._serialPortInterface_srlPort.write(bytes(completeMessage, "utf8"))
        except Exception as ex:
            # except any undocumented exceptions from writing
            printTraceback(ex)
            pass

    def waitForSafeReadWrite_srlPort(self, incrementFactor):
        """
        Waits until the time passed by the last call to this method passes.
        Stores the next time that interaction with the Silverpak is safe.
        Part of the lock group: srlPort.
        <param name="incrementFactor">How long to wait after this call to this method,
        expressed as a multiple of PortDelatUnit, typically 1.0.</param>
        """
        # wait until next read write time
        sleepyTime = max(0, self._nextReadWriteTime - time.time())
        time.sleep(sleepyTime)
        # increment next read write time
        self._nextReadWriteTime = time.time() + self.PortDelayUnit * incrementFactor


DTProtocolTxStartStr = "/"
DTProtocolTxEndStr = "R\r"
DTProtocolRxStartStr = "/0"
DTProtocolRxEndStr = "\x03"

DTProtocolComDataBits = 8
DTProtocolComParity = serial.PARITY_NONE
DTProtocolComStopBits = serial.STOPBITS_ONE

# Returns a complete message to write to the Silverpak.
def GenerateMessage(recipient, commandList):
    return DTProtocolTxStartStr + getDriverAddress(recipient) + commandList + DTProtocolTxEndStr

# Evaluates an RX string received from the Silverpak and returns whether the RX message is complete and valid.
def IsRxDataComplete(rxData):
    if rxData == None:
        return False
    rxStartPos = rxData.find(DTProtocolRxStartStr)
    if rxStartPos == -1:
        # rxData does not include Start
        return False
    return rxData.find(DTProtocolRxEndStr, rxStartPos + len(DTProtocolRxStartStr)) != -1

def TrimRxData(rxData):
    """Returns just the status char and data from the passed RX message. RX data must be complete."""
    start = rxData.find(DTProtocolRxStartStr)
    firstTrim = rxData[start + len(DTProtocolRxStartStr):]
    return firstTrim[:firstTrim.find(DTProtocolRxEndStr)]

def makeSerialPort():
    """Configures non-variable properties of the passed serial port object in accordance with DT Protocol."""
    srlPort = serial.Serial()
    srlPort.timeout = 0
    srlPort.bytesize = DTProtocolComDataBits
    srlPort.parity = DTProtocolComParity
    srlPort.stopbits = DTProtocolComStopBits
    return srlPort

def SearchComPorts(portName=Silverpak.DefaultPortname, baudRate=Silverpak.DefaultBaudRate, driverAddress=Silverpak.DefaultDriverAddress):
    """
    Searches for available Silverpaks and returns a PortInformation class for every serached COM port.
    if any parameters are not set, all possible values for the parameters will be attempted.
    This method can raise an ArgumentOutOfRangeException or an ValueError if passed values are invalid.
    """
    if portName == Silverpak.DefaultPortname:
        # Search all COM ports
        portNames = ["COM%i" % i for i in range(1, 10)]
    else:
        # Search a specific COM port
        portNames = [portName]
    return [SearchBaudRates(portName, baudRate, driverAddress) for portName in portNames]

def SearchBaudRates(portName, baudRate=Silverpak.DefaultBaudRate, driverAddress=Silverpak.DefaultDriverAddress):
    """
    Searches for an available Silverpak at the specified COM port.
    if any parameters are not set, all possible values for the parameters will be attempted.
    This method can raise an ArgumentOutOfRangeException or an ValueError if passed values are invalid.
    """
    portInfo = None
    if baudRate == Silverpak.DefaultBaudRate:
        # Search all baud rates
        baudRates = [9600, 19200, 38400]
    else:
        # Search specific baud rate
        baudRates = [baudRate]
    for baudRate in baudRates:
        portInfo = SearchDriverAddresses(portName, baudRate, driverAddress)
        if portInfo != None:
            break
    if portInfo == None:
        portInfo = PortInformation(portName=portName, portStatus = PortStatuses.Empty)
    return portInfo

def SearchDriverAddresses(portName, baudRate, driverAddress=Silverpak.DefaultDriverAddress):
    """
    Searches for an available Silverpak at the specified COM port with the specified baud rate.
    if any parameters are not set, all possible values for the parameters will be attempted.
    Returns null instead of a PortInformation with .PortStatus = Empty.
    This method can raise an ArgumentOutOfRangeException or an ValueError if passed values are invalid.
    """
    if driverAddress == Silverpak.DefaultDriverAddress:
        # Search all driver addresses
        driverAddresses = allDriverAddresses
    else:
        # Search specified driver address
        driverAddresses = [driverAddress]
    for driverAddress in driverAddresses:
        portInfo = GetSilverpakPortInfo(portName, baudRate, driverAddress)
        if portInfo != None:
            return portInfo
    return None

_nextSerialPortTimes_lock = threading.RLock()
_nextSerialPortTimes = {}
def GetSilverpakPortInfo(portName, baudRate, driverAddress):
    """
    Searches for an available Silverpak at the specified COM port with the specified baud rate and driver address.
    Returns null instead of a PortInformation with .PortStatus = Empty.
    This method can raise an ArgumentOutOfRangeException or an ValueError if passed values are invalid.
    """
    sp = makeSerialPort()
    sp.port = portName
    sp.baudrate = baudRate

    # delay if this port name has been used recently by this method
    with _nextSerialPortTimes_lock:
        now = time.time()
        if portName in _nextSerialPortTimes:
            # returning to the same port as at least once before
            sleepyTime = max(0, _nextSerialPortTimes[portName] - now)
        else:
            # first time using this port
            sleepyTime = 0
        nextTime = now + sleepyTime + SilverpakConnectionManager.PortDelayUnit
        _nextSerialPortTimes[portName] = nextTime
    time.sleep(sleepyTime)

    # test the COM port
    try:
        # Open the serial port. can raise errors.
        sp.open()
        # Write a safe query. can raise errors
        sp.write(bytes(GenerateMessage(driverAddress, SafeQueryCommandStr), "utf8"))
        # read response
        # accumulates chunks of RX data
        totalRx = ""
        while True:
            # wait for a chunk to be written to the read buffer
            time.sleep(SilverpakConnectionManager.PortDelayUnit)
            # retrieve any data from the read buffer
            newRx = str(sp.read(1), "mbcs")
            if newRx == "":
                # abort if no data was written
                return None
            totalRx += newRx
            # check to see if the RX data is complete
            if IsRxDataComplete(totalRx):
                break
        # success
        return PortInformation(
            portName=portName,
            baudRate=baudRate,
            driverAddress=driverAddress,
            portStatus=PortStatuses.AvailableSilverpak,
        )
    except serial.serialutil.SerialException as ex:
        # TODO: refine this error checking
        return PortInformation(portName=portName, portStatus=PortStatuses.Invalid)
    finally:
        # make sure the port is closed
        try:
            if sp.isOpen():
                sp.close()
        except:
            pass


class Commands:
    """
    All available commands.
    For more information, see
    http://www.linengineering.com/LinE/contents/stepmotors/pdf/Silverpak23C-256Commands.pdf
    """
    # Homing and Positioning
    GoHome = "Z"
    SetPosition = "z"
    GoAbsolute = "A"
    SetHomePolarity = "f"
    GoPositive = "P"
    GoNegative = "D"
    SetPulseJogDistance = "B"
    TerminateCommand = "T"
    SetMotorPolarity = "F"

    # Velocity and Acceleration
    SetVelocity = "V"
    SetAcceleration = "L"

    # Setting Current
    SetRunningCurrent = "m"
    SetHoldingCurrent = "h"

    # Looping and Branching
    BeginLoop = "g"
    EndLoop = "G"
    Delay = "M"
    HaltUntil = "H"
    SkipIf = "S"
    SetMode = "n"

    # Position Correction - Encoder Option Only
    SetEncoderMode = "N"
    SetPositionCorrectionTolerance = "aC"
    SetEncoderRatio = "aE"
    SetPositionCorrectionRetries = "au"
    RecoverEncoderTimeout = "r"

    # Program Stroage and Recall
    StoreProgram = "s"
    ExecuteStoredProgram = "e"

    # Program Execution
    RunCurrentCommand = "R"
    RepeatCurrentCommand = "X"

    # Microstepping
    SetMicrostepResolution = "j"
    SetMicrostepAdjust = "o"

    # On/Off Drivers (Outputs)
    SetOutputOnOff = "J"

    # Query Commands
    QueryMotorPosition = "?0"
    QueryStartVelocity = "?1"
    QuerySlewSpeed = "?2"
    QueryStopSpeed = "?3"
    QueryInputs = "?4"
    QueryCurrentVelocityModeSpeed = "?5"
    QueryMicrostepSize = "?6"
    QueryMicrostepAdjust = "?7"
    QueryEncoderPosition = "?8"
    ClearMemory = "?9"

    QueryCurrentCommand = "$"
    QueryFirmwareVersion = "&"
    QueryControllerStatus = "Q"
    TerminateCommands = "T"
    EchoNumber = "p"

    # Baud Control
    SetBaudRate = "b"

# a safe query.
SafeQueryCommandStr = Commands.QueryControllerStatus


class MotorStates:
    """States for the motor"""
    # Serial Port is closed.
    Disconnected = "[Disconnected]"
    # Connected and not moving
    Stopped = "[Stopped]"
    # In the process of moving to the zero position.
    InitializingCoordinates_moveToZero = "[InitializingCoordinates_moveToZero]"
    # The "official" homing command. should complete very quickly.
    InitializingCoordinates_calibrateHome = "[InitializingCoordinates_calibrateHome]"
    # In the process of aborting coordinate initialization.
    AbortingCoordinateInitialization = "[AbortingCoordinateInitialization]"
    # In the process of moving.
    Moving = "[Moving]"

class LogLevel:
    Silence = 0
    Error = 1
    Warning = 2
    Debug = 3
    Communication = 4
logLevel = LogLevel.Warning
def makeLogSomething(prefix, minLevel):
    def logSomething(message):
        if logLevel < minLevel:
            return
        print(prefix + message)
    return logSomething
error = makeLogSomething("ERROR: ", LogLevel.Error)
warning = makeLogSomething("WARNING: ", LogLevel.Warning)
debug = makeLogSomething("DEBUG: ", LogLevel.Debug)
communication = makeLogSomething("COMMUNICATION: ", LogLevel.Communication)

