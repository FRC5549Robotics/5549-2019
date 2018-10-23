'''
    2018 code for Team 5549 Gryphon Robotics. Used for Power Up season.
    This contains the necessary code to drive using tank-steering and
    enables the use of attachments (omnom, lift, climb) with basic
    autonomous procedures.
'''

# This robot dispenses the cubes. Compared to him, you are nothing.
import wpilib
from wpilib.drive import DifferentialDrive
from wpilib import DriverStation
from ctre import WPI_TalonSRX

'''
Logitech Joysticks
    Joystick 0 --> left motors(red label)
    Joystick 1 --> right motors(green label)
Xbox 360 Controller
    Joystick 0 --> attachment control

AXIS MAPPING
    0 - X-Axis (left)   |   4 - X-Axis (right)
    1 - Y-Axis (left)   |   5 - Y-Axis (right)
    2 - Trigger (left)  |
    3 - Trigger (right) |

BUTTON MAPPING 
    1 - A   |   5 - Bumper (left)  
    2 - B   |   6 - Bumper (right)  
    3 - X   |   7 - Back            
    4 - Y   |   8 - Start           
'''


class MyRobot(wpilib.IterativeRobot):

    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        self.frontRightMotor = WPI_TalonSRX(0)
        self.rearRightMotor = WPI_TalonSRX(1)
        self.frontLeftMotor = WPI_TalonSRX(2)
        self.rearLeftMotor = WPI_TalonSRX(3)

        # object that handles basic intake operations
        self.omnom_left_motor = wpilib.Spark(7)
        self.omnom_right_motor = wpilib.Spark(8)

        # object that handles basic lift operations
        self.liftMotor = wpilib.Spark(4)

        # object that handles basic climb operations
        self.winch1 = wpilib.Spark(5)
        self.winch2 = wpilib.Spark(6)

        # defining motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # setting up drive group for drive motors
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        # defining omnom motor groups
        self.omnom_left = wpilib.SpeedControllerGroup(self.omnom_left_motor)
        self.omnom_right = wpilib.SpeedControllerGroup(self.omnom_right_motor)

        # setting up omnom group for omnom motors
        self.omnom = DifferentialDrive(self.omnom_left, self.omnom_right)
        self.omnom.setExpiration(0.1)

        # defines timer for autonomous
        self.timer = wpilib.Timer()

        # joystick 0, 1, 2 on the driver station
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.stick = wpilib.Joystick(2)

        # initialization of the FMS
        self.DS = DriverStation.getInstance()
        self.PS = DriverStation.getInstance()

        # initialization of the camera server
        wpilib.CameraServer.launch()

        # initialization of the gyroscope
        self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro.calibrate()

        # initialization of the limit switch
        self.limitSwitch = wpilib.DigitalInput(1)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.reset()
        self.timer.start()

        self.gyro.reset()
        self.rightTargetHeading = -(self.gyro.getAngle() + 90.0)
        self.leftTargetHeading = -(self.gyro.getAngle() - 90.0)

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # gets randomization of field elements
        gameData = self.DS.getGameSpecificMessage()
        # gets location of robot on the field
        position = self.PS.getLocation()

        # basic autonomous function presets

        def stop_motor():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
        
        def straight_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .8, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .8, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 0, WPI_TalonSRX.DemandType.Neutral, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 1, WPI_TalonSRX.DemandType.Neutral, 0)
            self.omnom_left.set(-0.1)
            self.omnom_right.set(0.1)

        def straight_slow_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .55, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .55, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 0, WPI_TalonSRX.DemandType.Neutral, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 1, WPI_TalonSRX.DemandType.Neutral, 0)

        def straight_super_slow_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .45, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .45, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 0, WPI_TalonSRX.DemandType.Neutral, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 1, WPI_TalonSRX.DemandType.Neutral, 0)

        def reverse_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.75, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.75, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 0, WPI_TalonSRX.DemandType.Neutral, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 1, WPI_TalonSRX.DemandType.Neutral, 0)

        def reverse_slow_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.AuxPID, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 0, WPI_TalonSRX.DemandType.Neutral, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.Follower, 1, WPI_TalonSRX.DemandType.Neutral, 0)

        def left_turn_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)

        def right_turn_speed():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, .5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, -.5, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)

        def lift_activate():
            self.liftMotor.set(0.75)

        def lift_lower():
            self.liftMotor.set(0.1)

        def dispense_cube():
            self.omnom_left.set(-0.5)
            self.omnom_right.set(0.5)

        def HansZeTransmissionBroke():
            self.frontRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearRightMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.frontLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
            self.rearLeftMotor.set(WPI_TalonSRX.ControlMode.PercentOutput, 0, WPI_TalonSRX.DemandType.ArbitraryFeedForward, 0)
        
        def distanceCalc():
            distMeters = ((self.frontRightMotor.getQuadraturePosition() + self.frontLeftMotor.getQuadraturePosition())/2 / 4096) * #circumference#
            return distMeters

        ##################################################################################

        # Autonomous functions

        def left_switch():
            if self.timer.get() <= 4.0:
                straight_slow_speed()
            elif 6.0 <= self.timer.get() <= 8.0:
                if self.gyro.getAngle() > self.rightTargetHeading:
                    right_turn_speed()
                    if self.gyro.getAngle() <= self.rightTargetHeading:
                        stop_motor()
            elif 8.0 < self.timer.get() < 10.0:
                if self.limitSwitch.get() is False:  # check True/False value
                    straight_super_slow_speed()
                else:
                    stop_motor()
            elif 10.0 < self.timer.get() < 10.5:
                lift_activate()
            elif 11.5 < self.timer.get() < 12.0:
                lift_lower()
                dispense_cube()

        def right_switch():
            if self.timer.get() <= 4.0:
                straight_slow_speed()
            elif 6.0 <= self.timer.get() <= 8.0:
                if self.gyro.getAngle() < self.leftTargetHeading:
                    left_turn_speed()
                    if self.gyro.getAngle() >= self.leftTargetHeading:
                        stop_motor()
            elif 8.0 < self.timer.get() < 10.0:
                if self.limitSwitch.get() is False:  # check True/False value
                    straight_super_slow_speed()
                else:
                    stop_motor()
            elif 10.0 < self.timer.get() < 10.5:
                lift_activate()
            elif 11.5 < self.timer.get() < 12.0:
                lift_lower()
                dispense_cube()

        def center_straight():
            if self.timer.get() < 3.0:
                straight_slow_speed()
            elif self.timer.get() > 3.0:
                stop_motor()

        def center_switch():
            if self.timer.get() < 3.0:
                straight_slow_speed()
            elif 3.5 < self.timer.get() < 4.0:
                lift_activate()
            elif 5.0 < self.timer.get() < 5.5:
                lift_lower()
                dispense_cube()

        def straight():
            if self.timer.get() < 3.5:
                straight_slow_speed()
            elif self.timer.get() > 3.5:
                stop_motor()

        def test():
            if self.timer.get() < 5.0:
                dispense_cube()

        """
        (Below) Test auto methods. Use during troubleshooting
        def gyro_test():
            if self.timer.get() < 2.0:
                if self.gyro.getAngle() >= self.targetHeading:
                    right_turn_speed()
                    if self.gyro.getAngle() < self.targetHeading:
                        stop_motor()

        def limit_switch_test():
            if self.timer.get() < 10.0:
                if self.limitSwitch.get() is True:
                    straight_super_slow_speed()
                else:
                    stop_motor()
        """

        """
        (Below) Old auto methods. Use as backup if current auto fails

        def left_switch_old():
            if self.timer.get() < 1.85:
                straight_speed()
            elif 1.85 < self.timer.get() < 2.6:
                right_turn_speed()
            elif 2.6 < self.timer.get() < 2.7:
                lift_activate()
            elif 2.8 < self.timer.get() < 3.1:
                lift_lower()
            elif 3.1 < self.timer.get() < 3.35:
                self.omnom_left.set(-0.5)
                self.omnom_right.set(0.5)
            elif 4.35 < self.timer.get() < 5.0:
                lift_activate()
            elif 6.35 < self.timer.get() < 6.85:
                dispense_cube()
            elif 6.85 < self.timer.get() < 7.85:
                lift_lower()
            elif self.timer.get() > 7.85:
                stop_motor()

        def right_switch_old():
            if self.timer.get() < 1.85:
                straight_speed()
            elif 1.85 < self.timer.get() < 2.6:
                left_turn_speed()
            elif 2.6 < self.timer.get() < 2.7:
                lift_activate()
            elif 2.8 < self.timer.get() < 3.1:
                lift_lower()
            elif 3.1 < self.timer.get() < 3.35:
                self.omnom_left.set(-0.5)
                self.omnom_right.set(0.5)
            elif 4.35 < self.timer.get() < 5.0:
                lift_activate()
            elif 6.35 < self.timer.get() < 6.85:
                dispense_cube()
            elif 6.85 < self.timer.get() < 7.85:
                lift_lower()
            elif self.timer.get() > 7.85:
                stop_motor()
        """

        ##################################################################################

        # L-L-R
        if gameData == "LLR" and position == 1:
            left_switch()
        elif gameData == "LLR" and position == 2:
            center_straight()
        elif gameData == "LLR" and position == 3:
            straight()

        # L-R-L
        elif gameData == "LRL" and position == 1:
            left_switch()
        elif gameData == "LRL" and position == 2:
            center_straight()
        elif gameData == "LRL" and position == 3:
            straight()

        # R-L-L
        elif gameData == "RLL" and position == 1:
            straight()
        elif gameData == "RLL" and position == 2:
            center_straight()
        elif gameData == "RLL" and position == 3:
            right_switch()

        # R-L-R
        elif gameData == "RLR" and position == 1:
            straight()
        elif gameData == "RLR" and position == 2:
            center_straight()
        elif gameData == "RLR" and position == 3:
            right_switch()

        # R-R-R
        elif gameData == "RRR" and position == 1:
            straight()
        elif gameData == "RRR" and position == 2:
            center_straight()
        elif gameData == "RRR" and position == 3:
            right_switch()

        # L-L-L
        elif gameData == "LLL" and position == 1:
            left_switch()
        elif gameData == "LLL" and position == 2:
            center_straight()
        elif gameData == "LLL" and position == 3:
            straight()

        # Other situations
        else:
            straight()

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.drive.setSafetyEnabled(True)

        # toggles for speed control
        self.toggle = 0

        # divisors that divide robot speed
        self.divisor = 1.15  # 1.15 Ian  # 2.0 for Sam

        # the previous state of the joystick button
        # self.buttonWasHeld = False

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""

        # controller mapping for tank steering
        leftAxis = self.leftStick.getRawAxis(1)
        rightAxis = self.rightStick.getRawAxis(1)

        # speed control

        if self.leftStick.getRawButton(1):
            if self.toggle == 0:
                self.divisor = 2.0  # 2.0 for Ian   # 1.25 for Sam
                self.toggle = 1
            elif self.toggle == 1:
                self.divisor = 1.15  # 1.15 for Ian  # 2.0 for Sam
                self.toggle = 0

        # controller mapping for omnom operation

        left_omnom_stick = self.stick.getRawAxis(1) / 1.25
        right_omnom_stick = self.stick.getRawAxis(5) / 1.25

        # lift controller mapping with relative speed

        if self.stick.getRawAxis(3):
            self.liftMotor.set(self.stick.getRawAxis(3))
        elif self.stick.getRawAxis(2):
            self.liftMotor.set(-self.stick.getRawAxis(2))
        else:
            self.liftMotor.set(0)

        # climb controller mapping with relative speed

        if self.stick.getRawButton(4):
            self.winch1.set(0.85)
            self.winch2.set(0.85)
        elif self.stick.getRawButton(1):
            self.winch1.set(-0.85)
            self.winch2.set(-0.85)
        else:
            self.winch1.set(0)
            self.winch2.set(0)

        # drives intake system using tank steering
        self.omnom.tankDrive(left_omnom_stick, right_omnom_stick)

        # drives drive system using tank steering
        self.drive.tankDrive(-leftAxis / self.divisor, -rightAxis / self.divisor)


if __name__ == '__main__':
    wpilib.run(MyRobot)
