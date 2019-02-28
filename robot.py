'''
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics
'''

import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from ctre import *


class MyRobot(wpilib.TimedRobot):
    ''' values for navx'''

    def robotInit(self):
        ''' Initialization of robot objects. '''

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
        self.buttonBox = wpilib.Joystick(3)

        ''' Button Status'''
        self.buttonStatusOne = False
        self.buttonStatusTwo = False
        self.buttonStatusThree = False
        self.buttonStatusFour = False
        self.buttonStatusFive = False
        self.buttonStatusSix = False
        self.buttonStatusSeven = False

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)    # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        '''Smart Dashboard'''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')

        # Smart Dashboard classes
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        '''Sensors'''
        # Hall Effect Sensor
        self.Hall = wpilib.DigitalInput(7)

        '''Timer'''
        # Timer
        self.timer = wpilib.Timer()

        '''Camera'''
        # initialization of the HTTP camera
        wpilib.CameraServer.launch('vision.py:main')
        self.sd.putString("", "Top Camera")
        self.sd.putString(" ", "Bottom Camera")
        self.sd.putString("  ", "Connection")

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        self.liftEncoder.reset()

        self.buttonStatusOne = False
        self.buttonStatusTwo = False
        self.buttonStatusThree = False
        self.buttonStatusFour = False
        self.buttonStatusFive = False
        self.buttonStatusSix = False
        self.buttonStatusSeven = False

    def autonomousPeriodic(self):
        ''' Called periodically during autonomous. '''

        '''Test Methods'''
        def encoder_test():
            ''' Drives robot set encoder distance away '''
            self.rightPos = fabs(self.rightEncoder.getQuadraturePosition())
            self.leftPos = fabs(self.leftEncoder.getQuadraturePosition())
            self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
            if 0 <= self.distIn <= 72:
                self.drive.tankDrive(0.5, 0.5)
            else:
                self.drive.tankDrive(0, 0)

        def Diagnostics():
            ''' Smart Dashboard Tests'''
            self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
            self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
            self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)

            # Smart Dashboard diagnostics
            self.sd.putNumber("Right Encoder Speed: ", abs(self.rightEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Left Encoder Speed: ", abs(self.leftEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Lift Encoder: ", self.liftEncoder.getDistance())

        def Pressure():
            self.Compressor.start()

        def cargoOne():
            if self.liftEncoder.get() <= 133:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 133:
                self.lift.set(0.05)
                self.buttonStatus = False

        def cargoTwo():
            if self.liftEncoder.get() <= 270:   # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 270:
                self.lift.set(0.05)
                self.buttonStatus = False

        def cargoThree():
            if self.liftEncoder.get() <= 415:   # Cargo 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 415:
                self.lift.set(0.05)
                self.buttonStatus = False

        def hatchOne():
            if self.liftEncoder.get() <= 96:    # Hatch 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 96:
                self.lift.set(0.05)
                self.buttonStatus = False

        def hatchTwo():
            if self.liftEncoder.get() <= 237:   # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 237:
                self.lift.set(0.05)
                self.buttonStatus = False

        def hatchThree():
            if self.liftEncoder.get() <= 378:   # Hatch 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 378:
                self.lift.set(0.05)
                self.buttonStatus = False

        def liftEncoderReset():
            self.lift.set(0.01)
            if self.Hall.get() is True:
                self.liftEncoder.reset()

        ''' Button Status Toggle '''
        if self.buttonBox.getRawButtonPressed(1):
            self.buttonStatusOne = not self.buttonStatusOne
        elif self.buttonBox.getRawButtonPressed(2):
            self.buttonStatusTwo = not self.buttonStatusTwo
        elif self.buttonBox.getRawButtonPressed(3):
            self.buttonStatusThree = not self.buttonStatusThree
        elif self.buttonBox.getRawButtonPressed(4):
            self.buttonStatusFour = not self.buttonStatusFour
        elif self.buttonBox.getRawButtonPressed(5):
            self.buttonStatusFive = not self.buttonStatusFive
        elif self.buttonBox.getRawButtonPressed(6):
            self.buttonStatusSix = not self.buttonStatusSix
        elif self.buttonBox.getRawButtonPressed(7):
            self.buttonStatusSeven = not self.buttonStatusSeven

        ''' Button Box Level Mapping '''
        if self.buttonStatusOne is True:
            cargoThree()
        elif self.buttonStatusTwo is True:
            hatchThree()
        elif self.buttonStatusTwo is True:
            cargoTwo()
        elif self.buttonStatusTwo is True:
            hatchTwo()
        elif self.buttonStatusTwo is True:
            cargoOne()
        elif self.buttonStatusTwo is True:
            hatchOne()
        elif self.buttonStatusTwo is True:
            liftEncoderReset()

        ''' Test Execution '''
        if self.DS.getGameSpecificMessage() == "pressure":
            Pressure()
        elif self.DS.getGameSpecificMessage() == "diagnostics":
            Diagnostics()

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # lift encoder rest
        self.liftEncoder.reset()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''
        '''        
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

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "High Speed")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low Speed")

        # ejector state
        if self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Claw: ", "Closed")

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

        ''' 
        # four-bar control
        if self.xbox.getRawAxis(1):
            self.liftArm.set(-self.xbox.getRawAxis(1) / 5.5)
        else:
            self.liftArm.set(0)
        '''

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