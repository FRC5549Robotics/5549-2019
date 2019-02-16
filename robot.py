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
from navx import AHRS


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
        self.ahrs = AHRS.create_spi()
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
        self.frontRightMotor = WPI_TalonSRX(3)
        self.rearRightMotor = WPI_TalonSRX(2)
        self.frontLeftMotor = WPI_TalonSRX(0)
        self.rearLeftMotor = WPI_TalonSRX(1)

        # drive train encoders
        self.rightEncoder = self.frontRightMotor
        self.leftEncoder = self.frontLeftMotor

        # drive train motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # drive train drive group
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        ''' Victor SPX Initialization '''
        # lift motors
        self.liftOne = WPI_VictorSPX(0)
        self.liftTwo = WPI_VictorSPX(1)
        self.lift = wpilib.SpeedControllerGroup(self.liftOne, self.liftTwo)

        # lift arm motors
        self.liftArmOne = WPI_VictorSPX(2)
        self.liftArmTwo = WPI_VictorSPX(3)
        self.liftArm = wpilib.SpeedControllerGroup(self.liftArmOne, self.liftArmTwo)

        # cargo intake motor
        self.cargo = WPI_VictorSPX(4)

        #lift encoders
        self.liftEncoder = self.liftOne

        #lift arm encoders
        self.liftArmEncoder = self.liftArmOne

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
        self.ahrs.reset()

    def autonomousPeriodic(self):
        ''' Called periodically during autonomous. '''

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

        def navxTest():
            if abs(self.ahrs.getAngle()) < 90.0:
                self.drive.tankDrive(0.6, -0.6)
            else:
                self.drive.tankDrive(0, 0)

        if self.DS.getGameSpecificMessage() == "RRR":
            breakIn()

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''
        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # NavX reset
        self.ahrs.reset()

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

        ''' Smart Dashboard '''
        # Smart Dashboard diagnostics
        self.sd.putString("", "Diagnostics")
        self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
        self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
        self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)
        self.sd.putBoolean(" Autonomous?", self.DS.isAutonomous())
        self.sd.putBoolean(" FMS Connection", self.DS.isFMSAttached())
        self.sd.putNumber("Right Encoder Speed: ", abs(self.frontRightMotor.getQuadratureVelocity()))
        self.sd.putNumber("Left Encoder Speed: ", abs(self.frontLeftMotor.getQuadratureVelocity()))
        string = self.sd.getString("Number", "null")
        self.sd.putString("String: ", string)

        # Smart Dashboard encoder
        self.RL = abs(self.rearLeftMotor.getQuadratureVelocity())
        self.RR = abs(self.rearRightMotor.getQuadratureVelocity())
        self.encoderAverage = ((self.RL + self.RR) / 2)
        self.sd.putNumber("Average Encoder Speed: ", self.encoderAverage)

        # Smart Dashboard match info
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

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()

        # gear shifting
        if self.rightStick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.leftStick.getRawButton(1):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)

        # hatch panel claw
        if self.rightStick.getRawButton(12):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.rightStick.getRawButton(11):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)

        # hatch panel ejection
        if self.leftStick.getRawButton(12):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.leftStick.getRawButton(11):  # deject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if self.xbox.getRawAxis(3):
            self.lift.set(self.xbox.getRawAxis(3) / 2.0)
        elif self.xbox.getRawAxis(2):
            self.lift.set(0.25)
        else:
            self.lift.set(0)

        # lift arm control
        if self.xbox.getRawAxis(5):
            self.liftArm.set(self.xbox.getRawAxis(5) / 2.0)
        else:
            self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawAxis(1):
            self.cargo.set(self.xbox.getRawButton(1) / 1.5)
        elif self.xbox.getRawButton(4):
            self.cargo.set(-self.xbox.getRawButton(1) / 1.5)
        else:
            self.cargo.set(0)

        # controller mapping for tank steering
        rightAxis = self.rightStick.getRawAxis(1)
        leftAxis = self.leftStick.getRawAxis(1)

        # drives drive system using tank steering
        self.drive.tankDrive(-leftAxis, -rightAxis)


if __name__ == '__main__':
    wpilib.run(MyRobot)