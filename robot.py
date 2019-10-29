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

        ''' Sensors '''
        # hall effect
        self.bottomHall = wpilib.DigitalInput(7)
        self.topHall = wpilib.DigitalInput(4)

        # ultrasonic
        self.ultrasonic = wpilib.AnalogInput(2)
        self.cargoUltrasonic = wpilib.AnalogInput(3)

        ''' Controller Initialization and Mapping '''
        # joystick - 1 | controller - 2
        self.joystick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)

        ''' Pneumatic Button Status '''
        self.gearButtonStatus = Toggle(self.joystick, 1)
        self.compressorButtonStatus = Toggle(self.xbox, 9)
        self.cargoOneButtonStatus = Toggle(self.xbox, 1)
        self.cargoTwoButtonStatus = Toggle(self.xbox, 4)
        self.shipButtonStatus = Toggle(self.xbox, 2)

        # self.clawButtonStatus = Toggle(self.xbox, 3)
        # self.ejectorButtonStatus = Toggle(self.xbox, 2)

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidGear = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidClaw = wpilib.DoubleSolenoid(2, 3)
        self.DoubleSolenoidEjector = wpilib.DoubleSolenoid(4, 5)
        self.Compressor.start()     # starts compressor to intake air

        ''' Smart Dashboard '''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')

        # Smart Dashboard classes
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        ''' Timer '''
        self.timer = wpilib.Timer()

        ''' Camera '''
        # initialization of the HTTP camera
        wpilib.CameraServer.launch()

        ''' PID '''
        # PID settings
        self.kP = 0.03  # proportional
        self.kI = 0.0   # integral
        self.kD = 0.0   # derivative
        self.kF = 0.075   # feed-forward

        # PID initialization
        self.PIDLiftController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.liftEncoder, output=self.lift)
        self.PIDLiftController.setInputRange(0, 450)        # encoder values in this case
        self.PIDLiftController.setOutputRange(-0.1, 0.6)       # motor speed values in this case
        self.PIDLiftController.setAbsoluteTolerance(1.0)    # maximum speed tolerance
        self.PIDLiftController.setContinuous(True)

        self.encoderRate = 0

    def pidWrite(self, output):
        self.encoderRate = output

    def robotCode(self):

        """if self.cargoOneButtonStatus.on and self.cargoTwoButtonStatus.get() is False:
            self.PIDLiftController.setSetpoint(200)     # 200 encoder value for level 1 cargo height
            self.liftToHeight = True
        elif self.cargoOneButtonStatus.off:
            self.PIDLiftController.setSetpoint(0)
            self.liftToHeight = False

        if self.cargoTwoButtonStatus.on and self.cargoOneButtonStatus.get() is False:
            self.PIDLiftController.setSetpoint(385)
            self.liftToHeight = True
        elif self.cargoTwoButtonStatus.off:
            self.PIDLiftController.setSetpoint(0)
            self.liftToHeight = False"""

        if self.bottomHall.get() is False:      # false for hall effect sensor is actually true
            self.liftEncoder.reset()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        ''' Pneumatics Dashboard States '''
        # gear state
        if self.DoubleSolenoidGear.get() == 1:
            self.sd.putString("Gear Shift: ", "FAST!")
        elif self.DoubleSolenoidGear.get() == 2:
            self.sd.putString("Gear Shift: ", "Slow")

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

        # compressor toggle - press left joystick on xbox controller to toggle
        if self.compressorButtonStatus.on:
            self.Compressor.start()
        elif self.compressorButtonStatus.off:
            self.Compressor.stop()

        # gear shift toggle - press trigger on joystick to toggle
        if self.gearButtonStatus.on:
            self.DoubleSolenoidGear.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.gearButtonStatus.off:
            self.DoubleSolenoidGear.set(wpilib.DoubleSolenoid.Value.kReverse)

        # # claw toggle
        # if self.clawButtonStatus.on:
        #     self.DoubleSolenoidClaw.set(wpilib.DoubleSolenoid.Value.kForward)
        # elif self.clawButtonStatus.off:
        #     self.DoubleSolenoidClaw.set(wpilib.DoubleSolenoid.Value.kReverse)
        #
        # # ejector toggle
        # if self.ejectorButtonStatus.on:
        #     self.DoubleSolenoidEjector.set(wpilib.DoubleSolenoid.Value.kForward)
        # elif self.ejectorButtonStatus.off:
        #     self.DoubleSolenoidEjector.set(wpilib.DoubleSolenoid.Value.kReverse)


        self.sd.putNumber("Encoder", self.liftEncoder.get())

        ''' Victor SPX Control (Lift, Lift Arm, Cargo) '''
        # lift control - checks first to see if preset height buttons are on; if not, manual lift control is enabled
        if self.cargoTwoButtonStatus.get() != self.cargoOneButtonStatus.get():
            if not self.PIDLiftController.isEnabled():
                self.PIDLiftController.enable()
            if self.cargoOneButtonStatus.get():
                self.PIDLiftController.setSetpoint(180)             # cargo level 1
            if self.cargoTwoButtonStatus.get():
                self.PIDLiftController.setSetpoint(380)             # cargo level 2
            if self.shipButtonStatus.get():
                self.PIDLiftController.setSetpoint(240)             # cargo ship
        elif not(self.cargoTwoButtonStatus.get() and self.cargoOneButtonStatus.get()):
            if self.PIDLiftController.isEnabled():
                self.PIDLiftController.disable()
            if self.xbox.getRawButton(5):                           # hold button - left bumper on xbox
                if self.liftEncoder.getDistance() <= 230:
                    self.lift.set(0.06)
                else:
                    self.lift.set(0.07)
            elif self.xbox.getRawAxis(3) > .01 or self.xbox.getRawAxis(2) < -.01:   # up - right trigger on xbox
                self.lift.set(self.xbox.getRawAxis(3) * 0.90)
            elif self.xbox.getRawAxis(2) > .01 or self.xbox.getRawAxis(2) < -.01:   # down - left trigger on xbox
                self.lift.set(-self.xbox.getRawAxis(2) * 0.45)
            else:
                self.lift.set(0)
        else:
            self.lift.set(0)

        # four-bar control
        if self.xbox.getRawButton(6):                               # hold - right bumper on xbox
            self.liftArm.set(0.12)
        elif not self.xbox.getRawButton(6):                         # if hold is not engaged
            self.liftArm.set(-self.xbox.getRawAxis(1) * 0.40)
        else:
            self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawButton(7):                           # hold
            self.cargo.set(0.14)
        elif -0.1 <= self.xbox.getRawAxis(5) <= 0.1:
            self.cargo.set(0.14)
        else:                           # take in - right joystick on xbox
            self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for arcade steering
        self.driveAxis = self.joystick.getRawAxis(1)            # forward and backward axis on joystick
        self.rotateAxis = self.joystick.getRawAxis(2)           # left and right axis on joystick

        #
        if self.DoubleSolenoidGear.get() == 1:                  # if on high gear
            self.divisor = 1.0                                  # 100% of high speed
        elif self.DoubleSolenoidGear.get() == 2:                # if on low gear
            self.divisor = 1.0                                 # 100% of slow speed 85% before
        else:
            self.divisor = 1.0                                  # 100% speed regardless of gear

        self.drive.arcadeDrive(-self.driveAxis * self.divisor, self.rotateAxis * 0.75)

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # lift encoder reset
        self.liftEncoder.reset()

        # compressor
        self.Compressor.start()

    def autonomousPeriodic(self):
        ''' Called periodically during autonomous. '''
        self.sd.putBoolean("LIFT RESET ", self.bottomHall.get())

        '''Test Methods'''

        def Diagnostics():
            ''' Smart Dashboard Tests'''
            # self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
            self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
            self.sd.putBoolean(" Brown Out?", self.roboController.isBrownedOut)

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
