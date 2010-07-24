
import silverpak

motor = silverpak.SilverpakManager()
motor.baudRate = 9600
motor.driverAddress = silverpak.getDriverAddress(1)

print(motor.FindAndConnect())

