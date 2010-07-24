"""

module for controlling a Silverpak stepper motor over a serial port.

supported devices:
 - Silverpak 23CE
   - http://www.linengineering.com/LinE/contents/stepmotors/SilverPak_23CE.aspx
   - http://www.linengineering.com/LinE/contents/stepmotors/pdf/Silverpak17C-256uStepping.pdf
 - Silverpak 17CE
   - http://www.linengineering.com/LinE/contents/stepmotors/SilverPak_17CE.aspx
   - http://www.linengineering.com/LinE/contents/stepmotors/pdf/Silverpak17C-256uStepping.pdf
"""

import sys
import time, threading

try:
    import serial
except ImportError:
    sys.stderr.write("\ninstall this: http://pyserial.sourceforge.net/\n\n")
    raise


# Public classes
class SilverpakManager:
    """Provides an interface to a Lin Engineering Silverpak23CE stepper motor"""

    # Public Fields
    DefaultAcceleration = 500
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
    DefaultVelocity = 230000
    
    # configurable
    errorCallback = None
    
    def IsActive(self):
        """Returns a value indicating whether this SilverpakManager is actively connected to a Silverpak23CE"""
        with self._motor_lock:
            return self._motorState_motor != MotorStates.Disconnected
    def IsReady(self):
        """Returns a value indicating whether the motor is ready to accept a command (i.e. connected, initialized, and stopped)"""
        with self._motor_lock:
            return self._motorState_motor == MotorStates.Ready
    def position(self):
        return self._position

    def __init__(self):
        self._connectionManager_motor = SilverpakConnectionManager(m_components)
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
        self._positionUpdaterThread_posUpd = threading.Thread(target=positionUpdater_run)

        # Raised when the connection to the Silverpak23CE is lost.
        self.connectionLostHandler = []
        # Raised when the motor stops moving.
        self.stoppedMovingHandler = []
        # Raised when the motor's position changes. Read the Position property to get the position.
        self.positionChangedHandler = []

    # Public methods
    def Connect(self):
        """
        Attempts to connect to a Silverpak23CE. 
        The PortName, BaudRate, and DriverAddress properties must be set or an ArgumentException will be thrown. 
        To auto-detect these properties, see FindAndConnect(). 
        The IsActive property must return False when calling this method or an InvalidSilverpakOperationException will be thrown.
        """
        with self._motor_lock:
            # Validate state and connection properties
            if self._motorState_motor != MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is already active. Make sure the IsActive property returns False before calling this method.")
            if self.portName == self.DefaultPortname: raise ArgumentException("PortName property must be set before calling this method. See also FindAndConnect().")
            if self.baudRate == self.DefaultBaudRate: raise ArgumentException("BaudRate property must be set before calling this method. See also FindAndConnect().")
            if self.driverAddress == self.DefaultDriverAddress: raise ArgumentException("DriverAddress property must be set before calling this method. See also FindAndConnect().")

            # Initialize connection manager's properties
            self._connectionManager_motor.PortName = m_portName
            self._connectionManager_motor.BaudRate = m_baudRate
            self._connectionManager_motor.DriverAddress = m_driverAddress
            # Attempt to connect
            if self._connectionManager_motor.Connect():
                # Connection succeeded
                self._motorState_motor = MotorStates.Connected
                return True
            else:
                # Connection failed
                return False

    def FindAndConnect(self):
        """
        Attempts to find and connect to a Silverpak23CE. 
        if any of the properties PortName, BaudRate, and DriverAddress are set to their defaults, all of their possible values will be searched. 
        After a successful connection, these properties will be set to their discovered values. 
        The IsActive property must return False when calling this method or an InvalidSilverpakOperationException will be thrown.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor != MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is already active. Make sure the IsActive property returns False before calling this method.")

            # Get information for all COM ports being searched
            portInfos = self.SearchComPorts(self.portName, self.baudRate, self.driverAddress)
            # Search the list of information for an available Silverpak23CE
            for iPI in portInfos:
                if iPI.PortStatus == PortStatuses.AvailableSilverpak:
                    # Listed available Silverpak23CE found
                    # Initialize connection manager's properties
                    self._connectionManager_motor.PortName = iPI.PortName
                    self._connectionManager_motor.BaudRate = iPI.BaudRate
                    self._connectionManager_motor.DriverAddress = iPI.DriverAddress
                    # Attempt to connect
                    # This should only evaluate to Flase in the event that the Silverpak23CE was disconnected between the call to SearchComPorts and now
                    if self._connectionManager_motor.Connect(): 
                        # Connection succeeded
                        # Save connection properties
                        self.portName = iPI.PortName
                        self.baudRate = iPI.BaudRate
                        self.driverAddress = iPI.DriverAddress
                        self.motorState_motor = MotorStates.Connected
                        return True
                    # In the rare occasion that block is skipped, try: the next iPI
            # End of list was reached and no available Silverpak23CE was found
            return False

    def InitializeMotorSettings(self):
        """
        Initialization Step 1. 
        This method will set the motor settings as specified by this SilverpakManager's properties.
        This method does not cause the motor to move.
        The next step is InitializeSmoothMotion().
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor != MotorStates.Connected: raise InvalidSilverpakOperationException("Initialization methods must be called in the proper order.")

            # Send settings-initialization command
            self._connectionManager_motor.Write(GenerateMessage(self.driverAddress, self.generateFullInitCommandList()), 4.0)
            # Update state
            self._motorState_motor = MotorStates.InitializedSettings
    
    def ResendMotorSettings(self):
        """
        Call this method if any changes to the motor settings properties need to be applied.
        This method does not cause the motor to move.
        Calling this method will raise an InvalidSilverpakOperationException if the IsActive property returns False or if the motor is moving.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor == MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is not active.")
            if self._motorState_motor in (
                    MotorStates.InitializingCoordinates_moveToZero,
                    MotorStates.InitializingCoordinates_calibrateHome,
                    MotorStates.Moving,
                ):
                raise InvalidSilverpakOperationException("Cannot resend motor settings while the motor is moving.")
            # Send settings command
            self._connectionManager_motor.Write(GenerateMessage(self.driverAddress, self.generateResendInitCommandList()), 4.0)

    def InitializeSmoothMotion(self):
        """
        Initialization Step 2. 
        This method will send a small motion command five times to bypass any initialization quirks that some motors are prone to exhibit.
        This method causes the motor to move up to 5 microsteps in the positive direction and causes the motor to briefly produce a rapid tapping sound.
        The next step is InitializeCoordinates().
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor != MotorStates.InitializedSettings: raise InvalidSilverpakOperationException("Initialization methods must be called in the proper order.")
            
            # Send a small motion command five times
            smoothMotionInitMsg = GenerateMessage(self.driverAddress, GenerateCommand(Commands.GoPositive, "1"))
            for _ in range(5):
                self._connectionManager_motor.Write(smoothMotionInitMsg, 3.0)
            # Update state
            self._motorState_motor = MotorStates.InitializedSmoothMotion
    
    def InitializeCoordinates(self):
        """
        Initialization Step 3. 
        This method will send the motor looking for its upper limit switch so it can zero its coordinate system.
        This method causes the motor to move in the negative direction until it trips the upper limit switch.
        The next initialization step is to wait for the CoordinatesInitialized event or to wait for the IsReady property to return True.
        Calling this method out of order will raise an InvalidSilverpakOperationException.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor != MotorStates.InitializedSmoothMotion: raise InvalidSilverpakOperationException("Initialization methods must be called in the proper order.")

            self.moveToZero()
        # Now that the motor is moving, begin listening for position changes
        self.startPositionUpdater()
    
    def moveToZero(self):
        # move to zero in preparation for home calibration
        cmd = GenerateCommand(Commands.SetPosition, int(self.maxPosition * (self.encoderRatio / 1000.0)))
        cmd += GenerateCommand(Commands.SetEncoderRatio, self.encoderRatio)
        cmd += GenerateCommand(Commands.GoAbsolute, 0)
        message = GenerateMessage(self.driverAddress, cmd)
        self._connectionManager_motor.Write(message, 2.0)

        # Update state
        self._motorState_motor = MotorStates.InitializingCoordinates_moveToZero

    def GoInfinite(self, positive):
        """
        Sends the motor either to position 0 or to the position specified by the MaxPosition property.
        Calling this method before the motor has been fully initialized will raise an InvalidSilverpakOperationException.
        """
        self.GoToPosition({True: self.maxPosition, False: 0}[positive])

    def StopMotor(self):
        """
        Stops the motor.
        Calling this method when the IsActive property returns False will raise an InvalidSilverpakOperationException.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor == MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is not active.")

            # Send stop command
            stopMessage = GenerateMessage(m_driverAddress, GenerateCommand(Commands.TerminateCommand))
            self._connectionManager_motor.Write(stopMessage, 1.0)
            # Update state if applicable
            if self._motorState_motor in (
                    MotorStates.InitializingCoordinates_moveToZero,
                    MotorStates.InitializingCoordinates_calibrateHome,
                ):
                self._motorState_motor = MotorStates.AbortingCoordinateInitialization
    
    def GoToPosition(self, position):
        """
        Sends the motor to the passed position.
        Calling this method before the motor has been fully initialized will raise an InvalidSilverpakOperationException.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor not in (
                    MotorStates.Ready,
                    MotorStates.Moving,
                ):
                raise InvalidSilverpakOperationException("Motor is not fully initialized")
            # Send absolute motion command
            self._connectionManager_motor.Write(GenerateMessage(self.driverAddress, GenerateCommand(Commands.GoAbsolute, position)), 1.0)
            # Update state
            self._motorState_motor = MotorStates.Moving
    
    def Disconnect(self):
        """
        Terminates the connection to the Silverpak23CE and closes the COM port.
        Calling this method will raise an InvalidSilverpakOperationException if the IsActive property returns False or if the motor is moving.
        """
        with self._motor_lock:
            # Validate state
            if self._motorState_motor == MotorStates.Disconnected: raise InvalidSilverpakOperationException("Connection is not active.")
            if self._motorState_motor in (
                    MotorStates.InitializingCoordinates_moveToZero,
                    MotorStates.InitializingCoordinates_calibrateHome,
                    MotorStates.Moving,
                ):
                raise InvalidSilverpakOperationException("Disconnecting while the motor is moving is not allowed.")

            # Disconnect
            self._connectionManager_motor.Disconnect()
            # Update state
            self._motorState_motor = MotorStates.Disconnected
    
    def _notify(self, handlers, *args):
        for handler in handlers[:]:
            handler(*args)
    # Event raisers TODO
    def OnConnectionLost(self):
        self._notify(self.connectionLostHandlers)
    def OnCoordinatesInitializationAborted(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.InitializationAborted)
    def OnCoordinatesInitialized(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.Initialized)
    def OnPositionChanged(self):
        self._notify(self.positionChangedHandlers)
    def OnStoppedMoving(self):
        self._notify(self.stoppedMovingHandlers, StoppedMovingReason.Normal)
    
    # Private methods
    def Dispose(self):
        """Disposes this component."""
        self.stopPositionUpdater()
    
    # Makes sure the position updater thread is running.
    def startPositionUpdater(self):
        with m_posUpd_lock:
            m_keepPositionUpdaterRunning_posUpd = True # make sure the position updater thread doesn't cancel
            if not m_positionUpdaterThread_posUpd.IsAlive: # only activate it when it's not active
                if m_positionUpdaterThread_posUpd.ThreadState == ThreadState.Stopped: # if it's previously completed running
                    m_positionUpdaterThread_posUpd = Thread(target=positionUpdater_run) # reinstantiate the thread
                m_positionUpdaterThread_posUpd.Start() # start the thread

    def stopPositionUpdater(self):
        """Stops the position updater thread and makes sure it dies."""
        if threading.current_thread() ==  self._positionUpdaterThread_posUpd:
            # the position updater thread cannot stop itself; a thread can never see itself die.
            # stop the position updater thread on a seperate thread.
            threading.Thread(target=lambda: self.stopPositionUpdater_not_positionUpdaterThread()).Start()
        else:
            self.stopPositionUpdater_not_positionUpdaterThread()
    def stopPositionUpdater_not_positionUpdaterThread(self):
        """Stops the position updater thread and makes sure it dies. This method cannot be called on the position updater thread."""
        try:
            with self._posUpd_lock:
                # cancel the position updater
                self._keepPositionUpdaterRunning_posUpd = False
                if self._positionUpdaterThread_posUpd != None:
                    timeoutTime = time.time() + 1.0
                    while self._positionUpdaterThread_posUpd.is_alive():
                        time.sleep(0.1)
                        if timeoutTime <= time.time():
                            break
                    # make sure the position updater thread dies before releasing the lock
        except Exception as ex:
            SilverpakManager.invokeErrorCallback(ex)

    def positionUpdater_run(self):
        """Method that the position getter thread runs."""
        try:
            # check for cancelation
            while self._keepPositionUpdaterRunning_posUpd:
                # Keep time according to Environment.TickCount
                nextIterationTime = time.time() + self.positionUpdaterInterval / 1000.0
                # Update postion
                self.updatePosition()
                # Wait for the next iteration time
                time.sleep(Math.Max(0, nextIterationTime - time.time()))
        except Exception as ex:
            SilverpakManager.invokeErrorCallback(ex)
    
    def updatePosition():
        """Updates the Position property by querying the position of the motor."""
        # store a function to call after the lock has been released
        callbackAction = None
        try:
            with self._motor_lock:
                getPositionCmd = GenerateCommand(Commands.QueryMotorPosition)
                getPositionMessage = GenerateMessage(self.driverAddress, getPositionCmd)
                # make sure we know whether it's been set by starting it at an unlikely value
                newPosition = None
                response = None
                try:
                    self._homeCalibrationSteps
                except AttributeError:
                    self._homeCalibrationSteps = 0
                    self._failCount = 0
                try:
                    response = self._connectionManager_motor.WriteAndGetResponse(getPositionMessage, 1.0)
                except InvalidSilverpakOperationException:
                    # the SilverpakManager's been disconnected
                    # shut down updater thread
                    callbackAction = self.stopPositionUpdater
                    return
                # Serial Port is still active
                if response != None:
                    try:
                        newPosition = int(response)
                        # Got a valid response
                    except ValueError:
                        pass
                # track the number of times we didn't receive a valid response
                if self._position == newPosition:
                    # motor stopped moving
                    self._failCount = 0
                    if self._motorState_motor == MotorStates.InitializingCoordinates_moveToZero:
                        # wait! sometimes the motor will stop at 5000000 and lie about being at the top (stupid old firmware)
                        if Math.Abs(m_position - 5000000) < 100:
                            self.moveToZero()
                        else:
                            self._motorState_motor = MotorStates.InitializingCoordinates_calibrateHome
                            # Send the homing message
                            initCoordMessage = GenerateMessage(self.driverAddress, GenerateCommand(Commands.GoHome, int(self.maxPosition * (self.encoderRatio / 1000.0))))
                            self._connectionManager_motor.Write(initCoordMessage, 1.0)
                            self._homeCalibrationSteps = 0
                    elif self._motorState_motor == MotorStates.InitializingCoordinates_calibrateHome:
                        self._motorState_motor = MotorStates.Ready
                        callbackAction = self.OnCoordinatesInitialized
                    elif self._motorState_motor == MotorStates.AbortingCoordinateInitialization:
                        self._motorState_motor = MotorStates.InitializedSmoothMotion
                        callbackAction = self.OnCoordinatesInitializationAborted
                    elif self._motorState_motor == MotorStates.Moving:
                        self._motorState_motor = MotorStates.Ready
                        callbackAction = self.OnStoppedMoving
                elif newPosition != None:
                    # motor changed position
                    self._failCount = 0
                    self._position = newPosition
                    callbackAction = self.OnPositionChanged
                    # make sure the home calibration isn't sneaking away
                    if self._motorState_motor == MotorStates.InitializingCoordinates_calibrateHome:
                        self._homeCalibrationSteps += 1
                        if self._homeCalibrationSteps > 5:
                            # Calling shenanigans on initialization
                            # stop the motor damnit
                            stopMessage = GenerateMessage(self.driverAddress, GenerateCommand(Commands.TerminateCommand))
                            for _ in range(3):
                                self._connectionManager_motor.Write(stopMessage, 1.0)
                            # crash
                            sys.exit("Motor shenanigans detected! This is a quirk resulting from using outdated motor firmware.\nPlease restart the program.")
                else:
                    # failed to get a valid position
                    self._failCount += 1
                    if self._failCount >= 5:
                        # failed 5 times in a row. Silverpak23CE must no longer be available.
                        self._failCount = 0
                        # disconnect
                        self._motorState_motor = MotorStates.Disconnected
                        self._connectionManager_motor.Disconnect()
                        # raise LostConnection event
                        callbackAction = self.OnConnectionLost
        finally:
            # invoke callback sub if any
            if callbackAction != None:
                callbackAction()
    
    def generateFullInitCommandList(self):
        """Produces a command list to initialize the motor from scratch."""
        initMotorSettingsProgramHeader = GenerateCommand(Commands.SetPosition, "0")
        # Position Correction + Optical Limit Switches
        initMotorSettingsProgramFooter = GenerateCommand(Commands.SetMode, "10")
        return initMotorSettingsProgramHeader + self.generateResendInitCommandList() + initMotorSettingsProgramFooter
    
    def generateResendInitCommandList(self):
        """Produces a command list to set the adjustable motor settings."""
        return GenerateCommand(Commands.SetHoldCurrent, self.holdingCurrent) + \
                GenerateCommand(Commands.SetRunningCurrent, self.runningCurrent) + \
                GenerateCommand(Commands.SetMotorPolarity, self.motorPolarity) + \
                GenerateCommand(Commands.SetHomePolarity, self.homePolarity) + \
                GenerateCommand(Commands.SetPositionCorrectionTolerance, self.positionCorrectionTolerance) + \
                GenerateCommand(Commands.SetPositionCorrectionRetries, self.positionCorrectionRetries) + \
                GenerateCommand(Commands.SetEncoderRatio, "1000") + \
                GenerateCommand(Commands.SetVelocity, self.velocity) + \
                GenerateCommand(Commands.SetAcceleration, self.acceleration) + \
                GenerateCommand(Commands.SetEncoderRatio, self.encoderRatio)
    
    @staticmethod
    def invokeErrorCallback(ex):
        """Invokes the ErrorCalback delegate if it has been set. Otherwise, re-throws the exception so that the program crashes."""
        if SilverpakManager.errorCallback != None:
            SilverpakManager.errorCallback.Invoke(ex)
        else:
            raise ex

class InvalidSilverpakOperationException(Exception):
    """The exception that is thrown when a method call in namespace Silverpak23CE is invalid for the object's current state."""


class PortInformation:
    """Represents a collection of data for reporting the status of a COM port"""
    def __init__(self, partName=None, baudRate=0, portStatus=None, driverAddress=None):
        self.portName = portName
        self.baudRate = baudRate
        self.portStatus = portStatus
        self.driverAddress = driverAddress

searchableDriverAddresses = "@123456789:;<=>?"
def getDriverAddress(index):
    """0x0 <= index <= 0xf"""
    return searchableDriverAddresses[index]


class PortStatuses:
    # Indicates that there is an active, available Silverpak on this COM port
    AvailableSilverpak = "[available]"
    # Indicates that this COM port does not have an active Silverpak
    Empty = "[empty]"
    # Indicates that this COM port could not be read from or written to
    Invalid = "[invalid]"
    # Indicates that this COM port is already open by another resource
    Busy = "[busy]"


class StoppedMovingReason:
    """Represents the reason that the motor stopped moving."""
    # The motor stopped after a GoInfinite() or GoToPosition() command.
    Normal = "[normal]"
    # The InitializeCoordinates() command has completed without being interrupted.
    Initialized = "[init]"
    # The InitializeCoordinates() command is aborted by calling the StopMotor() method.
    InitializationAborted = "[init_abort]"


# Friend classes

class SilverpakConnectionManager:
    """Manages the connection to a Silverpak through a serial port."""

    # Public fields
    # The command string for a safe query.
    SafeQueryCommandStr = GenerateCommand(Commands.QueryControllerStatus, "")
    # The delay factor for a safe query.
    SafeQueryDelayFactor = 3.0
    # The minimum amount of time in milliseconds to wait for the Silverpak23CE to respond to a command.
    PortDelayUnit = 50 / 1000.0

    # Public properties

    # Public constructors
    def __init__(self):
        self.portName = None
        self.baudRate = 0
        self.driverAddress = None

        # Lock object for the serial port
        self._srlPort_lock = threading.RLock()
        # The serial port object used to communicate with a Silverpak.
        self._serialPortInterface_srlPort = InitializeSerialPort(SerialPort(components))

    # Public methods
    def Connect(self):
        """
        Attempts to connect to a Silverpak using the PortName, BaudRate, and DriverAddress properties. 
        Returns True if successful.
        Throws an InvalidSilverpakOperationException if already connected.
        """
        with self._srlPort_lock:
            # Validate SerialPort state
            if self._serialPortInterface_srlPort.IsOpen: raise InvalidSilverpakOperationException("Already connected.")
            try: # except all expected exceptions
                # apply serial port settings
                self._serialPortInterface_srlPort.PortName = m_portName
                self._serialPortInterface_srlPort.BaudRate = m_baudRate
                # Attempt to connect
                self._serialPortInterface_srlPort.Open()
                # Check for a Silverpak23CE
                response = self.writeAndGetResponse_srlPort(GenerateMessage(self.driverAddress, self.SafeQueryCommandStr), self.SafeQueryDelayFactor)
                if response != None:
                    return True
                else:
                    self.closeSerialPort_srlPort()
                    return False
            except ArgumentOutOfRangeException: pass # .BaudRate
            except ArgumentNullException: pass # .PortName
            except ArgumentException: pass # .PortName
            except UnauthorizedAccessException: pass # .Open
            except IO.IOException: pass # .Write (called from within writeAndGetResponse_srlPort())
            # Failed to connect. Make sure the SerialPort is closed
            self.closeSerialPort_srlPort()
            return False
    
    def Disconnect(self):
        """Makes sure there is no active connection to a Silverpak23CE."""
        with self._srlPort_lock:
            self.closeSerialPort_srlPort()

    def Write(completeMessage, delayFactor):
        """
        Writes the passed complete message to the Silverpak23CE.
        Throws an InvalidSilverpakOperationException if not connected.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak23CE is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        with m_srlPort_lock:
            # Validate state
            if not self._serialPortInterface_srlPort.IsOpen: raise InvalidSilverpakOperationException()
            # write message
            self.write_srlPort(completeMessage, delayFactor)

    def WriteAndGetResponse(self, completeMessage, delayFactor):
        """
        Writes the passed message to and returns the body of the response from the Silverpak23CE.
        if no response was received, returns null.
        Throws an InvalidSilverpakOperationException if not connected.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak23CE is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        with m_srlPort_lock:
            # Validate state
            if not m_serialPortInterface_srlPort.IsOpen: raise InvalidSilverpakOperationException()
            # write messag and get response
            return self.writeAndGetResponse_srlPort(completeMessage, delayFactor)

    def closeSerialPort_srlPort(self):
        if not self._serialPortInterface_srlPort.IsOpen:
            return
        try:
            # Close the serial port.
            self._serialPortInterface_srlPort.Close()
        except:
            # Ignore any exceptions that occure while closing.
            pass


    def writeAndGetResponse_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to and returns the body of the response from the Silverpak23CE.
        if no response was received, returns null.
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak23CE is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        # Clear the read buffer.
        self.safeReadExisting_srlPort(0.0)
        # Write the message.
        self.safeWrite_srlPort(completeMessage, delayFactor)
        # accumulates chunks of RX data
        totalRx = ""
        # Read the response from the Silverpak23CE in chunks until the accumulated message is complete.
        while True:
            # Read a chunk.
            rxStr = self.safeReadExisting_srlPort(1.0)
            if rxStr == None or rxStr == "":
                # if nothing came through, return null in lieu of an infinite loop.
                return None
            # Append chunk to accumulated RX data.
            totalRx += rxStr
            # check to see if the accumulated RX data is complete
            if IsRxDataComplete(totalRx):
                break
        # Trim the RX data. Garunteed to succeed because IsRxDataComplete(totalRx) returned True
        trimResponse = TrimRxData(totalRx)
        # return only the return data (not the Status Char).
        return trimResponse[1:]

    def write_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to the Silverpak23CE.
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak23CE is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        self.safeWrite_srlPort(completeMessage, delayFactor)

    def safeReadExisting_srlPort(self, delayFactor):
        """
        Reads the existing data on the read buffer from the Silverpak23CE after calling waitForSafeReadWrite_srlPort.
        In the event of an unexcepted exception from SerialPort.ReadExisting(), returns null.
        Part of the lock group: srlPort.
        <param name="delayFactor">How long to wait after reading from the Silverpak23CE,
        expressed as a multiple of PortDelatUnit, typically 1.0.</param>
        """
        # wait for safe read/write
        self.waitForSafeReadWrite_srlPort(delayFactor)
        try:
            return self._serialPortInterface_srlPort.ReadExisting()
        except:
            # except any undocumented exceptions from SerialPort.ReadExisting()
            return None

    def safeWrite_srlPort(self, completeMessage, delayFactor):
        """
        Writes the passed message to the Silverpak23CE after calling waitForSafeReadWrite_srlPort.
        Catches all exceptions from SerialPort.Write().
        Part of the lock group: srlPort.
        <param name="completeMessage">Recommended use generateMessage() to generate this parameter.</param>
        <param name="delayFactor">How long the the Silverpak23CE is expected to take to process the message, 
        expressed as a multiple of PortDelatUnit, typically in the range 1.0 to 3.0.</param>
        """
        # wait for safe read/write
        self.waitForSafeReadWrite_srlPort(delayFactor)
        try:
            self._serialPortInterface_srlPort.Write(completeMessage)
        except:
            # except any undocumented exceptions from SerialPort.Write()
            pass

    def waitForSafeReadWrite_srlPort(self, incrementFactor):
        """
        Waits until the time passed by the last call to this method passes.
        Part of the lock group: srlPort.
        <param name="incrementFactor">How long to wait after this call to this method,
        expressed as a multiple of PortDelatUnit, typically 1.0.</param>
        """
        # stores the next time that interaction with the Silverpak23CE is safe
        try:
            self._nextReadWriteTime
        except AttributeError:
            self._nextReadWriteTime = time.time()
        # wait until next read write time
        time.sleep(Math.Max(0, self._nextReadWriteTime - time.time()))
        # increment next read write time
        self._nextReadWriteTime = time.time() + self.PortDelayUnit * incrementFactor


# Consts and Functions for internal use
# The beginning of a sent message to a Silverpak23CE.
DTProtocolTxStartStr = "/"
# The end of a sent message to a Silverpak23CE.
DTProtocolTxEndStr = "R\r"
# The beginning of a received message from a Silverpak23CE.
DTProtocolRxStartStr = "/0"
# The end of a received message from a Silverpak23CE.
DTProtocolRxEndStr = "\x03"

# DataBits setting for operating a Silverpak23CE over a serial port.
DTProtocolComDataBits = 8
# Parity setting for operating a Silverpak23CE over a serial port.
DTProtocolComParity = serial.PARITY_NONE
# StopBits setting for operating a Silverpak23CE over a serial port.
DTProtocolComStopBits = STOPBITS_ONE
# Handshake setting for operating a Silverpak23CE over a serial port.
DTProtocolComHandshake = IO.Ports.Handshake.None

# Returns a complete message to write to the Silverpak23CE.
def GenerateMessage(recipient, commandList):
    """<param name="commandList">Recommended use GenerateCommand() to generate this parameter. Multiple commands can be concatenated and passed as this argument.</param>"""
    return DTProtocolTxStartStr + recipient + commandList + DTProtocolTxEndStr
def GenerateCommand(cmnd, operand=""):
    """Returns a command to pass to GenerateMessage()"""
    return GetCommandStr(cmnd) + operand

def GetDriverAddressStr(driver):
    """Returns the character to use in GenerateMessage()"""
    return chr(driver)

def GetCommandStr(command):
    """Returns the string used in GenerateCommand()"""
    return {
        # Homing and Positioning
        Commands.GoHome: "Z",
        Commands.SetPosition: "z",
        Commands.GoAbsolute: "A",
        Commands.SetHomePolarity: "f",
        Commands.GoPositive: "P",
        Commands.GoNegative: "D",
        Commands.SetPulseJogDistance: "B",
        Commands.TerminateCommand: "T",
        Commands.SetMotorPolarity: "F",

        # Velocity and Acceleration
        Commands.SetVelocity: "V",
        Commands.SetAcceleration: "L",

        # Setting Current
        Commands.SetRunningCurrent: "m",
        Commands.SetHoldCurrent: "h",

        # Looping and Branching
        Commands.BeginLoop: "g",
        Commands.EndLoop: "G",
        Commands.Delay: "M",
        Commands.HaltUntil: "H",
        Commands.SkipIf: "S",
        Commands.SetMode: "n",

        # Position Correction - Encoder Option Only
        Commands.SetEncoderMode: "N",
        Commands.SetPositionCorrectionTolerance: "aC",
        Commands.SetEncoderRatio: "aE",
        Commands.SetPositionCorrectionRetries: "au",
        Commands.RecoverEncoderTimeout: "r",

        # Program Stroage and Recall
        Commands.StoreProgram: "s",
        Commands.ExecuteStoredProgram: "e",

        # Program Execution
        Commands.RunCurrentCommand: "R",
        Commands.RepeatCurrentCommand: "X",

        # Microstepping
        Commands.SetMicrostepResolution: "j",
        Commands.SetMicrostepAdjust: "o",

        # On/Off Drivers (Outputs)
        Commands.SetOutputOnOff: "J",

        # Query Commands
        Commands.QueryMotorPosition: "?0",
        Commands.QueryStartVelocity: "?1",
        Commands.QuerySlewSpeed: "?2",
        Commands.QueryStopSpeed: "?3",
        Commands.QueryInputs: "?4",
        Commands.QueryCurrentVelocityModeSpeed: "?5",
        Commands.QueryMicrostepSize: "?6",
        Commands.QueryMicrostepAdjust: "?7",
        Commands.QueryEncoderPosition: "?8",
        Commands.ClearMemory: "?9",

        Commands.QueryCurrentCommand: "$",
        Commands.QueryFirmwareVersion: "&",
        Commands.QueryControllerStatus: "Q",
        Commands.TerminateCommands: "T",
        Commands.EchoNumber: "p",

        # Baud Control
        Commands.SetBaudRate: "b",
    }[command]


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
    fstTrim = rxData[start + len(DTProtocolRxStartStr)]
    return fstTrim[:fstTrim.find(DTProtocolRxEndStr)]

def InitializeSerialPort(srlPort):
    """Sets the DataBits, Parity, StopBits, and Handshake properties of the passed SerialPort object in accordance with DT Protocol."""
    srlPort.DataBits = DTProtocolComDataBits
    srlPort.Parity = DTProtocolComParity
    srlPort.StopBits = DTProtocolComStopBits
    srlPort.Handshake = DTProtocolComHandshake
    return srlPort

def SearchComPorts(portName=SilverpakManager.DefaultPortname, baudRate=SilverpakManager.DefaultBaudRate, driverAddress=SilverpakManager.DefaultDriverAddress):
    """
    Searches for available Silverpak23CE's and returns a PortInformation class for every serached COM port.
    if any parameters are not set, all possible values for the parameters will be attempted.
    This method can raise an ArgumentOutOfRangeException or an ArgumentException if passed values are invalid.
    """
    if portName == SilverpakManager.DefaultPortname:
        # Search all COM ports
        allPortNames = SerialPort.GetPortNames()
        rtnAry = []
        for portName in allPortNames:
            # Search this COM port
            rtnAry.append(SearchBaudRates(portNames, baudRate, driverAddress))
        return rtnAry
    else:
        # Search a specific COM port
        return [SearchBaudRates(portName, baudRate, driverAddress)]

def SearchBaudRates(portName, baudRate=SilverpakManager.DefaultBaudRate, driverAddress=SilverpakManager.DefaultDriverAddress):
    """
    Searches for an available Silverpak23CE at the specified COM port.
    if any parameters are not set, all possible values for the parameters will be attempted.
    This method can raise an ArgumentOutOfRangeException or an ArgumentException if passed values are invalid.
    """
    portInfo = None
    if baudRate == SilverpakManager.DefaultBaudRate:
        # Search all baud rates
        for baudRate in (9600, 19200, 38400):
            portInfo = SearchDriverAddresses(portName, baudRate, driverAddress)
            if portInfo != None:
                break
    else:
        # Search specific baud rate
        portInfo = SearchDriverAddresses(portName, baudRate, driverAddress)
    if portInfo == None:
        portInfo = PortInformation(portName=portName, portStatus = PortStatuses.Empty)
    return portInfo

def SearchDriverAddresses(portName, baudRate, driverAddress=SilverpakManager.DefaultDriverAddress):
    """
    Searches for an available Silverpak23CE at the specified COM port with the specified baud rate.
    if any parameters are not set, all possible values for the parameters will be attempted.
    Returns null instead of a PortInformation with .PortStatus = Empty.
    This method can raise an ArgumentOutOfRangeException or an ArgumentException if passed values are invalid.
    """
    if driverAddress == SilverpakManager.DefaultDriverAddress:
        # Search all driver addresses
        portInfo = None
        for driverAddress in searchableDriverAddresses:
            portInfo = GetSilverpakPortInfo(portName, baudRate, driverAddress)
            if portInfo != None:
                break
        return portInfo
    else:
        # Search specified driver address
        return GetSilverpakPortInfo(portName, baudRate, driverAddress)

_nextSerialPortTime = time.time()
def GetSilverpakPortInfo(portName, baudRate, driverAddress):
    """
    Searches for an available Silverpak23CE at the specified COM port with the specified baud rate and driver address.
    Returns null instead of a PortInformation with .PortStatus = Empty.
    This method can raise an ArgumentOutOfRangeException or an ArgumentException if passed values are invalid.
    """
    with InitializeSerialPort(SerialPort()) as sp:
        # set SerialPort parameters and allow exceptions to bubble out
        sp.PortName = portName
        sp.BaudRate = baudRate

        # delay if this method has been called recently
        time.sleep(Math.Max(0, _nextSerialPortTime - time.time()))
        _nextSerialPortTime = time.time() + SilverpakConnectionManager.PortDelayUnit

        # test the COM port
        try:
            # Open the serial port. can raise UnauthorizedAccessException
            sp.Open()
            # Write a safe query. can raise IOException
            sp.Write(GenerateMessage(driverAddress, SilverpakConnectionManager.SafeQueryCommandStr))
            # read response
            # accumulates chunks of RX data
            totalRx = ""
            while True:
                # wait for a chunk to be written to the read buffer
                time.sleep(SilverpakConnectionManager.PortDelayUnit)
                # retrieve any data from the read buffer
                newRx = sp.ReadExisting
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
        except UnauthorizedAccessException:
            # Port was already open
            return PortInformation(portName=portName, portStatus=PortStatuses.Busy)
        except IO.IOException:
            # Port was invalid (such as a Bluetooth virtual COM port)
            return PortInformation(portName=portName, portStatus=PortStatuses.Invalid)
        finally:
            # make sure the port is closed
            try:
                if sp.IsOpen: sp.Close()
            except:
                pass


# Friend enums
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
    SetHoldCurrent = "h"

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

class MotorStates:
    """States for the motor"""
    # Serial Port is closed.
    Disconnected = "[Disconnected]"
    # Serial Port is just open.
    Connected = "[Connected]"
    # Motor settings have been written to the Silverpak23CE.
    InitializedSettings = "[InitializedSettings]"
    # Small movements have been issued to the Silverpak23CE to clear initialization quirks.
    InitializedSmoothMotion = "[InitializedSmoothMotion]"
    # In the process of moving to the zero position.
    InitializingCoordinates_moveToZero = "[InitializingCoordinates_moveToZero]"
    # The "official" homing command. should complete very quickly.
    InitializingCoordinates_calibrateHome = "[InitializingCoordinates_calibrateHome]"
    # In the process of aborting coordinate initialization.
    AbortingCoordinateInitialization = "[AbortingCoordinateInitialization]"
    # Fully initialized and stopped.
    Ready = "[Ready]"
    # In the process of moving.
    Moving = "[Moving]"



