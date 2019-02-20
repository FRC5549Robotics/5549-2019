'''
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics
'''

import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard
from networktables import NetworkTables
from ctre import *


class MyRobot(wpilib.TimedRobot):
    ''' values for navx'''
    kP = 0.04
    kI = 0.00
    kD = 0.00
    kF = 0.00

    kToleranceDegrees = 2.0

    def robotInit(self):
        ''' Initialization of robot objects. '''

        ''' NavX '''
        #self.ahrs = AHRS.create_spi()
        '''
        turnController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.ahrs, output=self)
        turnController.setInputRange(-180.0, 180.0)
        turnController.setOutputRange(-0.6, 0.6)
        turnController.setAbsoluteTolerance(self.kToleranceDegrees)
        turnController.setContinuous(True)
        self.turnController = turnController
        self.rotateToAngleRate = 0
        '''

        ''' Talon SRX Initialization '''
        # drive train motors
        self.frontRightMotor = WPI_TalonSRX(4)
        self.rearRightMotor = WPI_TalonSRX(3)
        self.frontLeftMotor = WPI_TalonSRX(1)
        self.rearLeftMotor = WPI_TalonSRX(2)

        '''Encoders'''
        # drive train encoders
        self.rightEncoder = self.frontRightMotor
        self.leftEncoder = self.frontLeftMotor

        # lift encoders
        self.liftEncoder = wpilib.Encoder(8, 9)

        '''Motor Groups'''
        # drive train motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # drive train drive group
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        ''' Victor SPX Initialization '''
        # lift motors
        self.liftOne = WPI_VictorSPX(1)
        self.liftTwo = WPI_VictorSPX(2)
        self.lift = wpilib.SpeedControllerGroup(self.liftOne, self.liftTwo)

        # lift arm motors
        self.liftArmOne = WPI_VictorSPX(3)
        self.liftArmTwo = WPI_VictorSPX(4)
        self.liftArm = wpilib.SpeedControllerGroup(self.liftArmOne, self.liftArmTwo)

        # cargo intake motor
        self.cargo = WPI_VictorSPX(5)

        ''' Controller Initialization '''
        # joystick - 0, 1 | controller - 2
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)    # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')

        # Timer
        self.timer = wpilib.Timer()

        # Smart Dashboard classes
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        # initialization of the HTTP camera
        wpilib.CameraServer.launch()

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # NavX reset
        #self.ahrs.reset()

    def autonomousPeriodic(self):
        ''' Called periodically during autonomous. '''

        '''Test Methods'''
        def breakIn():
            if self.timer.get() <= 600:
                self.drive.tankDrive(1.0, 1.0)
            else:
                self.drive.tankDrive(0, 0)

        def encoder_test():
            self.rightPos = fabs(self.rightEncoder.getQuadraturePosition())
            self.leftPos = fabs(self.leftEncoder.getQuadraturePosition())
            self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
            if 0 <= self.distIn <= 72:
                self.drive.tankDrive(0.5, 0.5)
            else:
                self.drive.tankDrive(0, 0)

        def liftTest():
            if self.liftEncoder.get() <= 415:
                self.lift.set(0.5)
            elif self.liftEncoder.get() >= 415:
                self.lift.set(0.05)
            else:
                self.lift.set(0)

        def navxTest():
            if abs(self.ahrs.getAngle()) < 90.0:
                self.drive.tankDrive(0.6, -0.6)
            else:
                self.drive.tankDrive(0, 0)

        def Pressure():
            self.Compressor.start()

        if self.DS.getGameSpecificMessage() == "RRR":
            liftTest()

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # NavX reset
        # self.ahrs.reset()

        # lift encoder rest
        self.liftEncoder.reset()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''

        ''' NavX Control '''
        '''
        self.sd.putNumber("Gyro angle: ", self.ahrs.getAngle())
        rotateToAngle = False
        if self.xbox.getRawButton(7):
            self.ahrs.reset()
        elif self.xbox.getRawButton(4):
            self.turnController.setSetpoint(0.0)
            rotateToAngle = True
        elif self.xbox.getRawButton(2):
            self.turnController.setSetpoint(90.0)
            rotateToAngle = True
        elif self.xbox.getRawButton(3):
            self.turnController.setSetpoint(180.0)
            rotateToAngle = True
        elif self.xbox.getRawButton(1):
            self.turnController.setSetpoint(-90.0)
            rotateToAngle = True

        if rotateToAngle:
            self.turnController.enable()
            currentRotationRate = self.rotateToAngleRate
        else:
            self.turnController.disable()
            currentRotationRate = self.xbox.getX()
        '''

        ''' Smart Dashboard Tests'''
        '''
        self.sd.putString("", "Diagnostics")
        self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
        self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
        self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)
        self.sd.putBoolean(" Autonomous?", self.DS.isAutonomous())
        self.sd.putBoolean(" FMS Connection", self.DS.isFMSAttached())
        
        self.sd.putString(" ", "Match Info")
        self.sd.putString("Event Name: ", self.DS.getEventName())
        self.sd.putNumber("Match Time: ", self.timer.getMatchTime())
        self.sd.putNumber("Match Number: ", self.DS.getMatchTime())
        self.sd.putNumber("Location: ", self.DS.getLocation())
        if self.DS.getMatchType() == 3:
            self.sd.putString("Match Type: ", "Elimination")
        elif self.DS.getMatchType() == 1:
            self.sd.putString("Match Type: ", "Practice")
        elif self.DS.getMatchType() == 2:
            self.sd.putString("Match Type: ", "Qualification")
        else:
            self.sd.putString("Match Type: ", "None")

        if self.DS.getAlliance() == 0:
            self.sd.putString("Alliance: ", "Red")
        elif self.DS.getAlliance() == 1:
            self.sd.putString("Alliance: ", "Blue")
        else:
            self.sd.putString("Alliance: ", "Invalid")
        '''
        '''
        # Smart Dashboard diagnostics
        self.sd.putNumber("Right Encoder Speed: ", abs(self.frontRightMotor.getQuadratureVelocity()))
        self.sd.putNumber("Left Encoder Speed: ", abs(self.frontLeftMotor.getQuadratureVelocity()))
        self.sd.putNumber("Lift Encoder: ", self.liftEncoder.getDistance())
        
        # Smart Dashboard encoder
        self.RL = abs(self.rearLeftMotor.getQuadratureVelocity())
        self.RR = abs(self.rearRightMotor.getQuadratureVelocity())
        self.encoderAverage = ((self.RL + self.RR) / 2)
        self.sd.putNumber("Average Encoder Speed: ", self.encoderAverage)
        '''

        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "High Speed")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low  Speed")

        # claw state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        # ejector state
        if self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Ejector Pins: ", "Retracted")

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.rightStick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.leftStick.getRawButton(1):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(7):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(8):  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if self.xbox.getRawAxis(3):  # up
            self.lift.set(self.xbox.getRawAxis(3) / 1.75)
        elif self.xbox.getRawAxis(2):  # down
            self.lift.set(-self.xbox.getRawAxis(2) / 3.0)
        elif self.xbox.getRawButton(5):  # hold
            self.lift.set(0.05)
        else:
            self.lift.set(0)

        # # forebar control
        # if self.xbox.getRawAxis(1):
        #     self.liftArm.set(-self.xbox.getRawAxis(1) / 5.5)
        # else:
        #     self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawButton(1):   # take in
            self.cargo.set(1.0)
        elif self.xbox.getRawButton(4):  # push out
            self.cargo.set(-1.0)
        elif self.xbox.getRawButton(6):  # hold
            self.cargo.set(0.05)
        else:
            self.cargo.set(0)

        # controller mapping for tank steering
        rightAxis = self.rightStick.getRawAxis(1)
        leftAxis = self.leftStick.getRawAxis(1)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 0.90  # 90% of high speed
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 1.0  # normal slow speed
        else:
            self.divisor = 1.0

        self.drive.tankDrive(-leftAxis / self.divisor, -rightAxis/ self.divisor)  # drive divided by appropriate divisor


if __name__ == '__main__':
    wpilib.run(MyRobot)