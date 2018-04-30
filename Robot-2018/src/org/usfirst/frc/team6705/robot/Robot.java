/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Robot.java                        */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Elevator.ElevatorState;
import org.usfirst.frc.team6705.robot.Intake.IntakeState;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

//import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project
 */

public class Robot extends IterativeRobot {
    private String gameData;

    private static final String SWITCH_AUTO_1_CUBE = "singleSwitch";
    private static final String SWITCH_AUTO_2_CUBE = "doubleSwitch";
    private static final String SWITCH_AUTO_3_CUBE = "tripleSwitch";
    private static final String SCALE_SWITCH_AUTO = "scaleSwitch";
    private static final String DOUBLE_SCALE_AUTO = "doubleScale";
    private static final String BASELINE_AUTO = "baseline";
    // private static final String bestSimple = "bestSimple";
    private static final String MOTION_PROFILE_TEST = "mp";
    private static final String TEST = "test";
    private static final String SINGLE_SCALE = "singleScale";
    private static final String SCALE_AUTO_MP = "scaleMP";
    private static final String SWITCH_AUTO_MP = "switchMP";
    private static final String SWITCH_SCALE_AUTO_MP = "switchScaleMP";
    private String autoSelected;

    private String startingPosition;

    private SendableChooser<String> autoChooser = new SendableChooser<>();
    private SendableChooser<String> positionChooser = new SendableChooser<>();

    public static final Autonomous AUTO = new Autonomous();

    private boolean intakeOpen = false;
    private IntakeState intakeState = IntakeState.MANUAL;
    private double intakeStartTime = 0;
    private boolean intakeUp = false;
    private boolean actuateButtonPressed = false;

    private double distanceToLift = 0;
    private int direction = 1;
    private double previousHeight = FLOOR_HEIGHT;
    private int triggerIntervalsCounted = 0;
    private ElevatorState elevatorState = ElevatorState.MANUAL;

    public static final Timer TIMER = new Timer();

    private XboxController driveStick = new XboxController(DRIVE_STICK_CHANNEL);
    private Joystick liftStick = new Joystick(LIFT_STICK_CHANNEL);

    // Used for testing left and right sides of drive train
    private StringBuilder sbL = new StringBuilder();
    private StringBuilder sbR = new StringBuilder();

    // Used for logging data during testing or matches onto USB stick
    private String[] dataColumns = {"Timestamp (s)", "Left Position (Ft)", 
        "Right Position (Ft)", "Left Velocity (Ft/s)",
        "Right Velocity (Ft/s)", "Battery Voltage (V)" };
    // CSVLogger autoDataLogger = new CSVLogger("autoData", dataColumns);
    // CSVLogger teleopDataLogger = new CSVLogger("teleopData", dataColumns);

    private int loops = 0;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        System.out.println("Robot Init");

        Constants.getPreferences();
        /*
         * UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
         * camera.setResolution(320, 240); camera.setFPS(20);
         * camera.setExposureManual(35); camera.setBrightness(50);
         */

        autoChooser.addDefault("ONE cube on SWITCH", SWITCH_AUTO_1_CUBE);
        autoChooser.addObject("TWO cube on SWITCH", SWITCH_AUTO_2_CUBE);
        autoChooser.addObject("THREE cube on SWITCH", SWITCH_AUTO_3_CUBE);
        autoChooser.addObject("ONE cube on SCALE", SINGLE_SCALE);
        autoChooser.addObject("TWO cubes - scale AND switch", SCALE_SWITCH_AUTO);
        autoChooser.addObject("TWO cubes on SCALE", DOUBLE_SCALE_AUTO);
        // autoChooser.addObject("Best Simple Scoring Method", bestSimple);
        autoChooser.addObject("Cross Baseline ONLY", BASELINE_AUTO);
        autoChooser.addObject("Straight Motion Profile", MOTION_PROFILE_TEST);
        autoChooser.addObject("Test Auto", TEST);
        autoChooser.addObject("Double Scale with Motion Profiling", SCALE_AUTO_MP);
        autoChooser.addObject("Double Switch with Motion Profiling", SWITCH_AUTO_MP);
        autoChooser.addObject("Switch AND Scale with Motion Profiling", SWITCH_SCALE_AUTO_MP);
        // autoChooser.addObject("Stalling Test Auto", stall);
        SmartDashboard.putData("Auto choices", autoChooser);

        positionChooser.addDefault("Left Starting Position", LEFT);
        positionChooser.addObject("Middle Starting Position", MIDDLE);
        positionChooser.addObject("Right Starting Position", RIGHT);
        SmartDashboard.putData("Starting position", positionChooser);

        DriveTrain.setup();
        Elevator.setup();
        Intake.setup();

        // compressor.setClosedLoopControl(true);
        // compressor.start();

        DriveTrain.GYRO.calibrate();

    }

    @Override
    public void disabledInit() {
        AUTO.resetAuto();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autoSelected = autoChooser.getSelected();

        AUTO.resetAuto();
        Constants.getPreferences();
        DriveTrain.configPID();

        // autoDataLogger.reset();

        startingPosition = positionChooser.getSelected();
        System.out.print("Autonomous Init");
        System.out.print(autoSelected);

        SmartDashboard.putNumber("Auto State", 0);

        gameData = DriverStation.getInstance().getGameSpecificMessage();

        TIMER.reset();
        TIMER.start();
        // DriveTrain.gyro.setPIDSourceType(PIDSourceType.kDisplacement);//Remove this
        // line if it causes issues
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

        // Continue trying to get game data if it didn't show up immediately
        if (gameData.length() == 0 || gameData == null) {
            gameData = DriverStation.getInstance().getGameSpecificMessage();
        }

        updateSmartDashboard();

        // Log data to a .csv for analysis
        /*
         * double timeStamp = timer.get(); double leftPos =
         * convertTicksToFeet(DriveTrain.leftTalon.getSelectedSensorPosition(0)); double
         * rightPos =
         * convertTicksToFeet(DriveTrain.rightTalon.getSelectedSensorPosition(0));
         * double leftVel = getFPS(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
         * double rightVel = getFPS(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
         * double voltage = RobotController.getBatteryVoltage();
         */
        // String[] data = {"" + timeStamp, "" + leftPos,"" + rightPos, "" + leftVel, ""
        // + rightVel, "" + voltage};
        // autoDataLogger.writeLine(data);

        // Reset encoder if at floor
        if (Elevator.isAtFloor()) {
            Elevator.ENCODER.reset();
        }

        // Run state machine based on selected auto and starting position
        switch (autoSelected) {
        case SWITCH_AUTO_1_CUBE:
            AUTO.switchAuto1Cube((gameData.charAt(0) == 'L') ? 1 : -1, 
                    (gameData.charAt(1) == 'L') ? 1 : -1,
                    startingPosition);
            break;

        case SWITCH_AUTO_2_CUBE:
            AUTO.switchAuto2Cube((gameData.charAt(0) == 'L') ? 1 : -1,
                    (gameData.charAt(1) == 'L') ? 1 : -1,
                    startingPosition);
            break;

        case SWITCH_AUTO_3_CUBE:
            AUTO.switchAuto3Cube((gameData.charAt(0) == 'L') ? 1 : -1,
                    (gameData.charAt(1) == 'L') ? 1 : -1,
                    startingPosition);
            break;

        case SINGLE_SCALE:
            AUTO.singleScaleAuto((gameData.charAt(1) == 'L') ? 1 : -1, 
                    startingPosition);
            break;

        case BASELINE_AUTO:
            System.out.print("Baseline Auto Periodic");
            AUTO.baselineAuto((gameData.charAt(1) == 'L') ? 1 : -1,
                    startingPosition);
            break;

        case SCALE_SWITCH_AUTO:
            AUTO.scaleSwitchAuto((gameData.charAt(1) == 'L') ? 1 : -1,
                    (gameData.charAt(0) == 'L') ? 1 : -1,
                    startingPosition);
            break;

        case DOUBLE_SCALE_AUTO:
            AUTO.doubleScaleAuto((gameData.charAt(1) == 'L') ? 1 : -1,
                    startingPosition);
            break;
        case MOTION_PROFILE_TEST:
            AUTO.motionProfileTest();
            break;

        case SCALE_AUTO_MP:
            if (startingPosition == LEFT) {
                AUTO.scaleAutoMPLeft((gameData.charAt(1) == 'L') ? 1 : -1);
            } else if (startingPosition == RIGHT) {
                AUTO.scaleAutoMPRight((gameData.charAt(1) == 'L') ? 1 : -1);
            }
            break;

        case SWITCH_AUTO_MP:
            AUTO.switchAutoMP(startingPosition, (gameData.charAt(0) == 'L') ? 1 : -1);
            break;

        case SWITCH_SCALE_AUTO_MP:
            if (startingPosition == LEFT) {
                AUTO.scaleSwitchAutoLeftMP((gameData.charAt(1) == 'L') ? 1 : -1,
                        (gameData.charAt(0) == 'L') ? 1 : -1);
            } else if (startingPosition == RIGHT) {
                AUTO.scaleSwitchAutoRightMP((gameData.charAt(1) == 'L') ? 1 : -1,
                        (gameData.charAt(0) == 'L') ? 1 : -1);
            }
            break;

        case TEST:
            AUTO.testAuto();
            break;
        
        /*
         * case bestSimple: switch(startingPosition) { case left: if(gameData.charAt(1)
         * == 'L') { auto.scaleAuto(startingPosition, 1, (gameData.charAt(0) == 'L') ? 1
         * : -1); } else if(gameData.charAt(0) == 'L'){
         * auto.switchAuto(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1); }
         * else { auto.baselineAuto(); } break; case middle: if(gameData.charAt(0) ==
         * 'L') { auto.switchAuto(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 :
         * -1); } else { auto.switchAuto(startingPosition, -1, (gameData.charAt(1) ==
         * 'L') ? 1 : -1); } break; case right: if(gameData.charAt(1) == 'R') {
         * auto.scaleAuto(startingPosition, -1, (gameData.charAt(0) == 'L') ? 1 : -1); }
         * else if(gameData.charAt(0) == 'R') { auto.switchAuto(startingPosition, -1,
         * (gameData.charAt(1) == 'L') ? 1 : -1); } else { auto.baselineAuto(); } break;
         * } break;
         */
        default:
                
        }

    }

    @Override
    public void teleopInit() {

        DriveTrain.stop();
        Intake.stopRollers();

        DriveTrain.undoReverseDriveTrain();

        // teleopDataLogger.reset();

        DriveTrain.LEFT_TALON.configClosedloopRamp(RAMP_RATE_TELEOP, 0);
        DriveTrain.RIGHT_TALON.configClosedloopRamp(RAMP_RATE_TELEOP, 0);

        // If the intake is open due to whatever happened in auto, set intakeOpen to
        // true
        if (Intake.SOLENOID.get() == DoubleSolenoid.Value.kForward) {
            intakeOpen = true;
        } else if (Intake.SOLENOID.get() == DoubleSolenoid.Value.kReverse) {
            intakeOpen = false;
        }

        if (Elevator.isAtFloor()) {
            elevatorState = ElevatorState.MANUAL;
            Elevator.stop();
        }

        DriveTrain.setSpeed(0, 0);

    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

        operatorControl();
        // testDriveTrain();
        // updateSmartDashboard();
        /*
         * //Log data to a .csv for analysis double timeStamp = timer.get(); double
         * leftPos =
         * convertTicksToFeet(DriveTrain.leftTalon.getSelectedSensorPosition(0)); double
         * rightPos =
         * convertTicksToFeet(DriveTrain.rightTalon.getSelectedSensorPosition(0));
         * double leftVel = getFPS(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
         * double rightVel = getFPS(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
         * double voltage = RobotController.getBatteryVoltage();
         */
        // String[] data = {"" + timeStamp, "" + leftPos,"" + rightPos, "" + leftVel, ""
        // + rightVel, "" + voltage};
        // teleopDataLogger.writeLine(data);
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {

        testMotors();
        // testDriveTrain();

    }

    // operatorControl() is called periodically in both teleop and test periodic
    public void operatorControl() {
        updateSmartDashboard();

        double currentTime = TIMER.get();
        SmartDashboard.putNumber("Current Time", currentTime);

        // Joystick - control tank drive
        DriveTrain.tankDrive(driveStick.getY(GenericHID.Hand.kLeft),
                driveStick.getY(GenericHID.Hand.kRight));
        // DriveTrain.tankDrive(0.8, 0.8);

        // double brightness = liftStick.getRawAxis(3);
        // System.out.println("Brightness " + brightness);

        // Back button - reset encoders and gyro
        if (driveStick.getBackButton()) {
            DriveTrain.resetEncoders();
            DriveTrain.GYRO.reset();
            Elevator.ENCODER.reset();
        }

        double netTrigger = driveStick.getTriggerAxis(GenericHID.Hand.kRight)
                - driveStick.getTriggerAxis(GenericHID.Hand.kLeft);

        // Dpad and bumpers - control rollers and pneumatics

        if (Math.abs(netTrigger) > 0.05) {
            Intake.roll(netTrigger);
        } else if (driveStick.getBumper(GenericHID.Hand.kRight)) {
            pickUpCube();
            System.out.println("Bumper right, pick up");
        } else if (driveStick.getBumper(GenericHID.Hand.kLeft)) {
            dropCube();
        } else if (liftStick.getRawButton(1)) {
            Intake.intake();
        } else {
            Intake.stopRollers();
        }

        // Actuating intake - toggle on side button (2) of big joystick
        if (liftStick.getRawButton(2) && intakeUp && !actuateButtonPressed) {
            Intake.actuateDown();
            actuateButtonPressed = true;
            intakeUp = false;
        } else if (liftStick.getRawButton(2) && !intakeUp && !actuateButtonPressed) {
            Intake.actuateUp();
            intakeUp = true;
            actuateButtonPressed = true;
        } else if (!liftStick.getRawButton(2)) {
            actuateButtonPressed = false;
        }

        /*
         * if (driveStick.getPOV(dPadChannel) > 325 || (driveStick.getPOV(dPadChannel) <
         * 35 && driveStick.getPOV(dPadChannel) >= 0)) { intakeState =
         * IntakeState.MANUAL; System.out.println("Dpad up"); rollOut(); } else if
         * (driveStick.getPOV(dPadChannel) > 145 && driveStick.getPOV(dPadChannel) <
         * 215) { intakeState = IntakeState.MANUAL; rollIn(); }
         */

        // Buttons - set Elevator lift to certain height - floor, switch, or scale
        if (((liftStick.getRawButton(11) || liftStick.getRawButton(12) 
                || driveStick.getAButton())
                && elevatorState != ElevatorState.FLOOR) && TIMER.get() < 145) {
            System.out.println("Move to floor button pressed");
            moveToFloor();
        } else if (((liftStick.getRawButton(9) || liftStick.getRawButton(10) 
                || driveStick.getBButton())
                && elevatorState != ElevatorState.SWITCH) && TIMER.get() < 145) {
            moveToSwitch();
        } else if (((liftStick.getRawButton(7) || liftStick.getRawButton(8) 
                || driveStick.getYButton())
                && elevatorState != ElevatorState.SCALE) && TIMER.get() < 145) {
            moveToScale();
        } else if (TIMER.get() > 145 && elevatorState != ElevatorState.FLOOR) {
            moveToFloor();
        }

        /*
         * //Control Intake Angle if (liftStick.getRawButton(11) ||
         * liftStick.getRawButton(12)) { Intake.angleDown(); } else if
         * (liftStick.getRawButton(10) || liftStick.getRawButton(9)) {
         * Intake.angleDiagonal(); } else if (liftStick.getRawButton(8) ||
         * liftStick.getRawButton(7)) { Intake.angleUp(); }
         */

        // Operator Joystick - lift and lower Elevator

        double joystickValue = -liftStick.getRawAxis(1);
        // double netTrigger = driveStick.getTriggerAxis(GenericHID.Hand.kRight) -
        // driveStick.getTriggerAxis(GenericHID.Hand.kLeft);
        // System.out.println("Net Trigger: " + netTrigger);
        if (Math.abs(joystickValue) >= 0.15 && TIMER.get() < 145) {
            triggerIntervalsCounted += 1;
            previousHeight = Elevator.getCurrentPosition();
            Elevator.setTeleop(joystickValue, triggerIntervalsCounted);
            elevatorState = ElevatorState.MANUAL;
        } else if (driveStick.getXButton() && liftStick.getRawButton(2)) {
            elevatorState = ElevatorState.MANUAL;
            Elevator.MOTOR.set(-0.9);
        } else if (elevatorState == ElevatorState.MANUAL && !Elevator.isAtFloor()) {
            triggerIntervalsCounted = 0;
            Elevator.maintainHeight(previousHeight);
        } else if (elevatorState == ElevatorState.MANUAL) {
            triggerIntervalsCounted = 0;
            Elevator.stop();
        } else {
            triggerIntervalsCounted = 0;
        }

        // Start button - deploy ramps at end of game
        if (TIMER.get() >= 120 && driveStick.getStartButton()) {
            deployRamps();
        }

        // *********************************************************************//

        // Check Elevator State
        if (Elevator.isAtFloor()) {
            Elevator.ENCODER.reset();
        }

        if (elevatorState == ElevatorState.FLOOR) {
            // double currentHeight = Elevator.getCurrentPosition();
            if (Elevator.isAtFloor()) { // Within desired range, stop elevating
                System.out.println("AT FLOOR, STOP");
                Elevator.stop();
                Elevator.ENCODER.reset();
                previousHeight = FLOOR_HEIGHT;
                elevatorState = ElevatorState.MANUAL;
            } else {
                System.out.println("GOING TO FLOOR");
                Elevator.moveToHeight(FLOOR_HEIGHT, distanceToLift, direction);
            }
        }

        if (elevatorState == ElevatorState.SWITCH) {
            double currentHeight = Elevator.getCurrentPosition();

            if (currentHeight < SWITCH_HEIGHT + ELEVATOR_TOLERANCE 
                    && currentHeight > SWITCH_HEIGHT - ELEVATOR_TOLERANCE) {
                // Within desired range, stop elevating
                // Elevator.stop();
                previousHeight = SWITCH_HEIGHT;
                elevatorState = ElevatorState.MANUAL;
                System.out.println("STOP SWITCH");
            } else {
                Elevator.moveToHeight(SWITCH_HEIGHT, distanceToLift, direction);
                System.out.println("SWITCH MOVE");
            }
        }

        if (elevatorState == ElevatorState.SCALE) {

            double currentHeight = Elevator.getCurrentPosition();

            if (currentHeight < SCALE_HEIGHT + ELEVATOR_TOLERANCE 
                    && currentHeight > SCALE_HEIGHT - ELEVATOR_TOLERANCE) {
                // Within desired range, stop elevating
                System.out.println("STOP SCALE");
                // Elevator.stop();
                previousHeight = SCALE_HEIGHT;
                elevatorState = ElevatorState.MANUAL;
            } else {
                System.out.println("SCALE MOVE");
                Elevator.moveToHeight(SCALE_HEIGHT, distanceToLift, direction);
            }
        }

        // *********************************************************************//

        /*
         * //Check Intake State if (intakeState == IntakeState.INTAKING) { if
         * (currentTime - intakeStartTime >= timeToRollIn) { //Stop based on time, or
         * maybe a limit switch if implement Intake.stopRollers(); intakeState =
         * IntakeState.MANUAL; } else { Intake.intake(); } }
         *
         * if (intakeState == IntakeState.OUTTAKING) { if (currentTime - intakeStartTime
         * >= timeToRollOut) { Intake.stopRollers(); intakeState = IntakeState.MANUAL; }
         * else { Intake.outtake(); } }
         */

        // *********************************************************************//

    }

    // Left Bumper
    public void pickUpCube() {
        // intakeStartTime = timer.get();
        // intakeState = IntakeState.INTAKING;
        Intake.close();
        Intake.intake();
        // intakeOpen = false;
    }

    // Right Bumper
    public void dropCube() {
        intakeStartTime = TIMER.get();
        // intakeState = IntakeState.OUTTAKING;
        Intake.open();
        intakeOpen = true;
    }

    /*
     * //Dpad up public void rollOut() { Intake.outtake(); }
     *
     * //Dpad down public void rollIn() { Intake.intake(); }
     */

    // A Button
    public void moveToFloor() {
        // Elevator.setHeight(floorHeight);
        elevatorState = ElevatorState.FLOOR;
        direction = -1;
        previousHeight = Elevator.getCurrentPosition();
        distanceToLift = Math.abs(Elevator.getCurrentPosition() - FLOOR_HEIGHT);
    }

    // B Button
    public void moveToSwitch() {
        // Elevator.setHeight(switchHeight);
        elevatorState = ElevatorState.SWITCH;
        direction = (Elevator.getCurrentPosition() < SWITCH_HEIGHT) ? 1 : -1;
        previousHeight = Elevator.getCurrentPosition();
        distanceToLift = Math.abs(Elevator.getCurrentPosition() - SWITCH_HEIGHT);
    }

    // Y Button
    public void moveToScale() {
        // Elevator.setHeight(scaleHeight);
        elevatorState = ElevatorState.SCALE;
        direction = (Elevator.getCurrentPosition() < SCALE_HEIGHT) ? 1 : -1;
        previousHeight = Elevator.getCurrentPosition();
        distanceToLift = Math.abs(Elevator.getCurrentPosition() - SCALE_HEIGHT);
    }

    // Start Button
    public void deployRamps() {
        Ramps.deploy();
    }

    public void updateSmartDashboard() {
        SmartDashboard.putNumber("Encoder position left",
                DriveTrain.LEFT_TALON.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Encoder position right",
                DriveTrain.RIGHT_TALON.getSelectedSensorPosition(0));

        SmartDashboard.putNumber("Motor Output Left",
                DriveTrain.LEFT_TALON.getMotorOutputPercent());
        SmartDashboard.putNumber("Motor Output Right",
                DriveTrain.RIGHT_TALON.getMotorOutputPercent());

        /*
         * SmartDashboard.putNumber("Right MP position target",
         * DriveTrain.rightTalon.getActiveTrajectoryPosition());
         * SmartDashboard.putNumber("Left MP position target",
         * DriveTrain.leftTalon.getActiveTrajectoryPosition());
         * SmartDashboard.putNumber("Right MP velocity target",
         * DriveTrain.leftTalon.getActiveTrajectoryVelocity());
         * SmartDashboard.putNumber("Left MP velocity target",
         * DriveTrain.rightTalon.getActiveTrajectoryVelocity());
         */

        SmartDashboard.putNumber("Motor velocity left",
                DriveTrain.LEFT_TALON.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Motor velocity right",
                DriveTrain.RIGHT_TALON.getSelectedSensorVelocity(0));

        SmartDashboard.putNumber("Left closed loop error",
                DriveTrain.LEFT_TALON.getClosedLoopError(0));
        SmartDashboard.putNumber("Right closed loop error",
                DriveTrain.RIGHT_TALON.getClosedLoopError(0));

        SmartDashboard.putNumber("Gyro Value", DriveTrain.getGyro());
        SmartDashboard.putNumber("Elevator Encoder Count", Elevator.ENCODER.get());
        SmartDashboard.putNumber("Elevator Current Height From Ground",
                Elevator.getCurrentPosition());
        SmartDashboard.putBoolean("Is At Floor Limit Switch?", Elevator.isAtFloor());
        String state = "Manual";
        if (elevatorState == ElevatorState.FLOOR) {
            state = "Floor";
        } else if (elevatorState == ElevatorState.SWITCH) {
            state = "Switch";
        } else if (elevatorState == ElevatorState.SCALE) {
            state = "Scale";
        }
        SmartDashboard.putString("Elevator State", state);
        String intake = "Open";
        if (Intake.SOLENOID.get() == DoubleSolenoid.Value.kReverse) {
            intake = "Closed";
        }
        SmartDashboard.putString("Intake Pneumatic Status", intake);
    }

    public void testMotors() {
        if (driveStick.getAButton()) {
            Elevator.SPARK_1.set(0.5);
        } else {
            Elevator.SPARK_1.set(0);
        }

        if (driveStick.getBButton()) {
            Elevator.SPARK_2.set(0.5);
        } else {
            Elevator.SPARK_2.set(0);
        }

        if (driveStick.getYButton()) {
            DriveTrain.LEFT_TALON.set(0.5);
        } else {
            DriveTrain.LEFT_TALON.set(0.5);
        }

        if (driveStick.getXButton()) {
            DriveTrain.RIGHT_TALON.set(0.5);
        } else {
            DriveTrain.RIGHT_TALON.set(0.5);
        }
    }

    public void testDriveTrain() {

        double leftYStick = driveStick.getY(GenericHID.Hand.kLeft);
        if (leftYStick < -0.93) {
            leftYStick = -1.0;
        }

        double motorOutputLeft = DriveTrain.LEFT_TALON.getMotorOutputPercent();
        double motorOutputRight = DriveTrain.RIGHT_TALON.getMotorOutputPercent();

        sbL.append("LOut: ");
        sbR.append("ROut: ");
        sbL.append(motorOutputLeft);
        sbR.append(motorOutputRight);

        sbL.append("LSpd: ");
        sbR.append("RSpd: ");
        sbL.append(DriveTrain.LEFT_TALON.getSelectedSensorVelocity(0));
        sbR.append(DriveTrain.RIGHT_TALON.getSelectedSensorVelocity(0));

        if (driveStick.getAButton()) {
            double targetVelocity = leftYStick * convertVelocity(11);
            DriveTrain.LEFT_TALON.set(ControlMode.Velocity, targetVelocity);
            DriveTrain.RIGHT_TALON.set(ControlMode.Velocity, targetVelocity);

            sbL.append("LErr:");
            sbR.append("RErr:");
            sbL.append(DriveTrain.LEFT_TALON.getClosedLoopError(0));
            sbR.append(DriveTrain.RIGHT_TALON.getClosedLoopError(0));

            sbL.append("LTarg:");
            sbR.append("RTarg:");
            sbL.append(targetVelocity);
            sbR.append(targetVelocity);
        } else {
            DriveTrain.LEFT_TALON.set(ControlMode.PercentOutput, leftYStick);
            DriveTrain.RIGHT_TALON.set(ControlMode.PercentOutput, leftYStick);
        }

        if (++loops >= 10) {
            loops = 0;
            System.out.println(sbL.toString());
            System.out.println(sbR.toString());
        }

        sbL.setLength(0);
        sbR.setLength(0);
    }

}