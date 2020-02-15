'''
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics
'''
import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from ctre import *
from robotpy_ext.control.toggle import Toggle


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        ''' Initialization of robot objects. '''

        ''' Talon SRX Initialization '''
        # drive train motors
        self.frontRightMotor = WPI_TalonSRX(4)
        self.rearRightMotor = WPI_TalonSRX(3)
        self.frontLeftMotor = WPI_TalonSRX(1)
        self.rearLeftMotor = WPI_TalonSRX(2)

        ''' Motor Groups '''
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

        ''' Encoders '''
        # drive train encoders
        self.rightEncoder = self.frontRightMotor
        self.leftEncoder = self.frontLeftMotor

        # lift encoder
        self.liftEncoder = wpilib.Encoder(8, 9)
        # liftArm encoder
        self.liftArmEncoder = wpilib.Encoder(5, 6, True)

        ''' Sensors '''
        # Hall Effect Sensor
        self.minHall = wpilib.DigitalInput(7)
        self.maxHall = wpilib.DigitalInput(4)
        self.limitSwitch = wpilib.DigitalInput(3)
        self.ultrasonic = wpilib.AnalogInput(2)
        self.cargoUltrasonic = wpilib.AnalogInput(3)

        ''' Controller Initialization and Mapping '''
        # joystick - 0, 1 | controller - 2
        self.joystick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)

        ''' Pneumatic Button Status '''
        self.clawButtonStatus = Toggle(self.xbox, 2)
        self.gearButtonStatus = Toggle(self.joystick, 1)
        self.ejectorPinButtonStatus = Toggle(self.xbox, 1)
        self.compressorButtonStatus = Toggle(self.xbox, 9)
        self.liftHeightButtonStatus = Toggle(self.xbox, 3)

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)    # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        ''' Smart Dashboard '''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')
        self.sd.putString("  ", "Connection")

        # Smart Dashboard classes
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        ''' Timer '''
        self.timer = wpilib.Timer()

        ''' Camera '''
        # initialization of the HTTP camera
        wpilib.CameraServer.launch('vision.py:main')
        self.sd.putString("", "Top Camera")
        self.sd.putString(" ", "Bottom Camera")

        ''' PID settings for lift '''
        self.kP = 0.03
        self.kI = 0.0
        self.kD = 0.0
        self.kF = 0.1

        self.PIDLiftcontroller = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.liftEncoder, output=self)
        self.PIDLiftcontroller.setInputRange(0, 400)
        self.PIDLiftcontroller.setOutputRange(-0.5, 0.5)
        self.PIDLiftcontroller.setAbsoluteTolerance(1.0)
        self.PIDLiftcontroller.setContinuous(True)

        self.encoderRate = 0

    def pidWrite(self, output):
        self.encoderRate = output

    def robotCode(self):

        if self.liftHeightButtonStatus.on:
            self.PIDLiftcontroller.setSetpoint(200)
            self.liftToHeight = True
        elif self.liftHeightButtonStatus.off:
            self.PIDLiftcontroller.setSetpoint(0)
            self.liftToHeight = False

        def hatchOne():
            if self.liftEncoder.getDistance() < 80:  # Hatch 2
                self.lift.set(0.3)
            elif self.liftEncoder.getDistance() >= 80:
                self.lift.set(0.07)

        def hatchTwo():
            if self.liftEncoder.getDistance() < 275:   # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 275:
                self.lift.set(0.07)

        def cargoOne():
            if self.liftEncoder.getDistance() < 150:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 150:
                self.lift.set(0.05)

        def cargoTwo():
            if self.liftEncoder.getDistance() < 320:  # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 320:
                self.lift.set(0.05)

        def cargoShip():
            if self.liftEncoder.getDistance() < 280:  # Cargo ship
                self.lift.set(0.5)
            elif self.liftEncoder.getDistance() >= 280:
                self.lift.set(0.07)

        # ''' Button Box Level Mapping '''
        # if self.buttonStatusOne.on:
        #     # hatchOne()
        #     cargoOne()
        # elif self.buttonStatusTwo.on:  # comment out for hatch
        #     cargoTwo()
        # elif self.buttonStatusThree.on:
        #     # hatchTwo()
        #     cargoShip()

        if self.minHall.get() is False:
            self.liftEncoder.reset()

        if self.limitSwitch.get() is False:
            self.liftArmEncoder.reset()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        ''' Pneumatics Dashboard States '''
        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "HIGH SPEED!!!")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low")

        # ejector state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        ''' Ultrasonic Range Detection '''
        # robot ultrasonic
        self.ultraValue = self.ultrasonic.getVoltage()
        if 0.142 <= self.ultraValue <= 0.146:
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!")
        else:
            self.sd.putString("PLAYER STATION RANGE: ", "NO!")

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()
        if 0.70 <= self.cargoUltraValue <= 1.56:
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE")
        else:
            self.sd.putString("HATCH RANGE: ", "NOT IN RANGE")

        ''' Pneumatics Toggles '''

        # Compressor
        if self.compressorButtonStatus.on:
            self.Compressor.start()
        elif self.compressorButtonStatus.off:
            self.Compressor.stop()

        # Claw Toggle
        if self.clawButtonStatus.on:
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)  # open claw
        elif self.clawButtonStatus.off:
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)  # close claw

        # Ejector Pins Toggle
        if self.ejectorPinButtonStatus.on:
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)  # eject
        elif self.ejectorPinButtonStatus.off:
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)  # retract

        # Gear Shift Toggle
        if self.gearButtonStatus.on:
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)  # shift right
        elif self.gearButtonStatus.off:
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)  # shift left

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if self.liftHeightButtonStatus.get() is False:
            if self.xbox.getRawButton(5):  # hold
                self.lift.set(0.05)
            elif self.xbox.getRawAxis(3):  # up
                self.lift.set(self.xbox.getRawAxis(3) * 0.85)
            elif self.xbox.getRawAxis(2):  # down
                self.lift.set(-self.xbox.getRawAxis(2) * 0.45)
            else:
                self.lift.set(0)
        # else:
        #     if self.liftToHeight is True:
        #         self.PIDLiftcontroller.enable()
        #         self.liftHeight = self.encoderRate
        #         self.lift.set(self.liftHeight)
        #     else:
        #         self.PIDLiftcontroller.disable()
        #         self.lift.set(0)

        # # four-bar control
        # if self.xbox.getRawButton(6):   # hold
        #     self.liftArm.set(0.12)
        # elif not self.xbox.getRawButton(6):
        #     self.liftArm.set(-self.xbox.getRawAxis(1) * 0.35)
        # else:
        #     self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawButton(7):   # hold
            self.cargo.set(0.12)
        elif self.xbox.getRawAxis(5):  # take in
            self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for arcade steering
        self.driveAxis = self.joystick.getRawAxis(1)
        self.rotateAxis = self.joystick.getRawAxis(2)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 1.0  # 90% of high speed
            self.turnDivisor = 0.8
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 0.85  # normal slow speed
            self.turnDivisor = 0.75
        else:
            self.divisor = 1.0

        if self.driveAxis != 0:
            self.leftSign = self.driveAxis / fabs(self.driveAxis)
        else:
            self.leftSign = 0

        if self.rotateAxis != 0:
            self.rightSign = self.rotateAxis / fabs(self.rotateAxis)
        else:
            self.rightSign = 0

        self.drive.arcadeDrive(-self.driveAxis * self.divisor, self.rotateAxis * 0.75)

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        self.liftEncoder.reset()
        self.liftArmEncoder.reset()

    def autonomousPeriodic(self):

        self.sd.putBoolean("LIFT RESET ", self.minHall.get())

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

        ''' Test Execution '''
        if self.DS.getGameSpecificMessage() == "pressure":
            Pressure()
        elif self.DS.getGameSpecificMessage() == "diagnostics":
            Diagnostics()

        self.robotCode()

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # lift encoder rest
        self.liftEncoder.reset()

        # compressor
        self.Compressor.start()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''

        self.robotCode()


if __name__ == '__main__':
    wpilib.run(MyRobot)
