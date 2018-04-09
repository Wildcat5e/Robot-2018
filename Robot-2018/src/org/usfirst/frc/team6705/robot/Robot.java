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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Elevator;
import org.usfirst.frc.team6705.robot.Elevator.ElevatorState;
import org.usfirst.frc.team6705.robot.Intake.IntakeState;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
	
	private static final String switchAuto1Cube = "singleSwitch";
	private static final String switchAuto2Cube = "doubleSwitch";
	private static final String scaleSwitchAuto = "scaleSwitch";
	private static final String doubleScaleAuto = "doubleScale";
	private static final String baselineAuto = "baseline";
	//private static final String bestSimple = "bestSimple";
	private static final String motionProfileTest = "mp";
	private static final String test = "test";
	private static final String singleScale = "singleScale";
	private static final String scaleAutoMP = "scaleMP";
	private static final String switchAutoMP = "switchMP";
	private static final String switchScaleAutoMP = "switchScaleMP";
	private String autoSelected;
	
	private String startingPosition;
	
	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> positionChooser = new SendableChooser<>();
	
	public static Autonomous auto = new Autonomous();
	
	boolean intakeOpen = false; 
	IntakeState intakeState = IntakeState.MANUAL;
	double intakeStartTime = 0;
	boolean intakeUp = false;
	boolean actuateButtonPressed = false;
	
	double distanceToLift = 0;
	int direction = 1;
	double previousHeight = floorHeight;
	int triggerIntervalsCounted = 0;
	ElevatorState elevatorState = ElevatorState.MANUAL;
	
	public static Timer timer = new Timer();
	
	XboxController driveStick = new XboxController(driveStickChannel);
	Joystick liftStick = new Joystick(liftStickChannel);
	
	//Used for testing left and right sides of drive train
	StringBuilder sbL = new StringBuilder();
	StringBuilder sbR  = new StringBuilder();
	
	//Used for logging data during testing or matches onto USB stick
	String[] dataColumns = {"Timestamp (s)", "Left Position (Ft)", "Right Position (Ft)", "Left Velocity (Ft/s)", "Right Velocity (Ft/s)", "Battery Voltage (V)"};
	//CSVLogger autoDataLogger = new CSVLogger("autoData", dataColumns);
	//CSVLogger teleopDataLogger = new CSVLogger("teleopData", dataColumns);
	
	int loops = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("Robot Init");
		
		Constants.getPreferences();
		/*
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(320, 240);
		camera.setFPS(20);
		camera.setExposureManual(35);
		camera.setBrightness(50);*/
		
		autoChooser.addDefault("ONE cube on SWITCH", switchAuto1Cube);
		autoChooser.addObject("TWO cube on SWITCH", switchAuto2Cube);
		autoChooser.addObject("ONE cube on SCALE", singleScale);
		autoChooser.addObject("TWO cubes - scale AND switch", scaleSwitchAuto);
		autoChooser.addObject("TWO cubes on SCALE (only in playoffs or if teammate is doing switch)", doubleScaleAuto);
		//autoChooser.addObject("Best Simple Scoring Method", bestSimple);
		autoChooser.addObject("Cross Baseline ONLY", baselineAuto);
		autoChooser.addObject("Straight Motion Profile", motionProfileTest);
		autoChooser.addObject("Test Auto", test);
		autoChooser.addObject("Double Scale with Motion Profiling", scaleAutoMP);
		autoChooser.addObject("Double Switch with Motion Profiling", switchAutoMP);
		autoChooser.addObject("Switch AND Scale with Motion Profiling", switchScaleAutoMP);
		//autoChooser.addObject("Stalling Test Auto", stall);
		SmartDashboard.putData("Auto choices", autoChooser);
		
		positionChooser.addDefault("Left Starting Position", left);
		positionChooser.addObject("Middle Starting Position", middle);
		positionChooser.addObject("Right Starting Position", right);
		SmartDashboard.putData("Starting position", positionChooser);
		
		DriveTrain.setup();
		Elevator.setup();
		Intake.setup();
		
		
		//compressor.setClosedLoopControl(true);
		//compressor.start();
		
		DriveTrain.gyro.calibrate();

	}
	
	@Override
	public void disabledInit() {
		auto.resetAuto();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = autoChooser.getSelected();
		
		auto.resetAuto();
		Constants.getPreferences();
		DriveTrain.configPID();
		
		//autoDataLogger.reset();
		
		startingPosition = positionChooser.getSelected();
		System.out.print("Autonomous Init");
		System.out.print(autoSelected);
		
		SmartDashboard.putNumber("Auto State", 0);
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		timer.reset();
		timer.start();
		//DriveTrain.gyro.setPIDSourceType(PIDSourceType.kDisplacement);//Remove this line if it causes issues
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		//Continue trying to get game data if it didn't show up immediately
		if (gameData.length() == 0 || gameData == null) {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		
		updateSmartDashboard();
		
		//Log data to a .csv for analysis
		double timeStamp = timer.get();
		double leftPos = convertTicksToFeet(DriveTrain.leftTalon.getSelectedSensorPosition(0));
		double rightPos = convertTicksToFeet(DriveTrain.rightTalon.getSelectedSensorPosition(0));
		double leftVel = getFPS(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		double rightVel = getFPS(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		double voltage = RobotController.getBatteryVoltage();
		
		String[] data = {"" + timeStamp, "" + leftPos,"" + rightPos, "" + leftVel, "" + rightVel, "" + voltage};
		//autoDataLogger.writeLine(data);
		
		//Reset encoder if at floor
		if (Elevator.isAtFloor()) {
			Elevator.encoder.reset();
		}
		
		//Run state machine based on selected auto and starting position
		switch (autoSelected) {
		case switchAuto1Cube:
			if (gameData.charAt(0) == 'L') {
				auto.switchAuto1Cube(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1); //Note: side 1 means left, side -1 means right
			} else {
				auto.switchAuto1Cube(startingPosition, -1, (gameData.charAt(1) == 'L') ? 1 : -1);
			}
			break;
		case switchAuto2Cube:
			if (gameData.charAt(0) == 'L') {
				auto.switchAuto2Cube(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1); //Note: side 1 means left, side -1 means right
			} else {
				auto.switchAuto2Cube(startingPosition, -1, (gameData.charAt(1) == 'L') ? 1 : -1);
			}
			break;
		case singleScale:
			auto.singleScaleAuto((gameData.charAt(1) == 'L') ? 1 : -1, startingPosition);
			break;
		case baselineAuto:
			System.out.print("Baseline Auto Periodic");
			auto.baselineAuto((gameData.charAt(1) == 'L') ? 1 : -1, startingPosition);
			break;
			
		case scaleSwitchAuto:
			auto.scaleSwitchAuto((gameData.charAt(1) == 'L') ? 1 : -1, (gameData.charAt(0) == 'L') ? 1 : -1, startingPosition);
			break;
			
		case doubleScaleAuto:
			auto.doubleScaleAuto((gameData.charAt(1) == 'L') ? 1: -1, startingPosition);
			break;
		case motionProfileTest:
			auto.motionProfileTest();
			break;
		case scaleAutoMP:
			if (startingPosition == left) {
				auto.scaleAutoMPLeft((gameData.charAt(1) == 'L') ? 1: -1);
			} else if (startingPosition == right) {
				auto.scaleAutoMPRight((gameData.charAt(1) == 'L') ? 1: -1);
			}
			break;
		case switchAutoMP:
			auto.switchAutoMP(startingPosition, (gameData.charAt(0) == 'L') ? 1: -1);
			break;
		case switchScaleAutoMP:
			if (startingPosition == left) {
				auto.scaleSwitchAutoLeftMP((gameData.charAt(1) == 'L') ? 1: -1, (gameData.charAt(0) == 'L') ? 1: -1);
			} else if (startingPosition == right) {
				auto.scaleSwitchAutoRightMP((gameData.charAt(1) == 'L') ? 1: -1, (gameData.charAt(0) == 'L') ? 1: -1);
			}
			break;
		case test:
			auto.testAuto();
			break;
			
			/*
			case bestSimple:
				switch(startingPosition) {
				case left:
					if(gameData.charAt(1) == 'L') {
						auto.scaleAuto(startingPosition, 1, (gameData.charAt(0) == 'L') ? 1 : -1);
					} else if(gameData.charAt(0) == 'L'){
						auto.switchAuto(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1);
					} else {
						auto.baselineAuto();
					}
					break;
				case middle:
					if(gameData.charAt(0) == 'L') {
						auto.switchAuto(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1);
					} else {
						auto.switchAuto(startingPosition, -1, (gameData.charAt(1) == 'L') ? 1 : -1);
					}
					break;
				case right:
					if(gameData.charAt(1) == 'R') {
						auto.scaleAuto(startingPosition, -1, (gameData.charAt(0) == 'L') ? 1 : -1);
					} else if(gameData.charAt(0) == 'R') {
						auto.switchAuto(startingPosition, -1,  (gameData.charAt(1) == 'L') ? 1 : -1);
					} else {
						auto.baselineAuto();
					}
					break;
				}
				break;*/
		}
		

	}
	
	@Override
	public void teleopInit() {
		
		DriveTrain.stop();
		Intake.stopRollers();
		
		DriveTrain.undoReverseDriveTrain();
		
		//teleopDataLogger.reset();
		
		DriveTrain.leftTalon.configClosedloopRamp(rampRateTeleop, 0);
		DriveTrain.rightTalon.configClosedloopRamp(rampRateTeleop, 0);
		
		//If the intake is open due to whatever happened in auto, set intakeOpen to true
		if (Intake.solenoid.get() == DoubleSolenoid.Value.kForward) {
			intakeOpen = true;
		} else if (Intake.solenoid.get() == DoubleSolenoid.Value.kReverse) {
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
		//testDriveTrain();
		//updateSmartDashboard();
		
		//Log data to a .csv for analysis
		double timeStamp = timer.get();
		double leftPos = convertTicksToFeet(DriveTrain.leftTalon.getSelectedSensorPosition(0));
		double rightPos = convertTicksToFeet(DriveTrain.rightTalon.getSelectedSensorPosition(0));
		double leftVel = getFPS(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		double rightVel = getFPS(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		double voltage = RobotController.getBatteryVoltage();
		
		String[] data = {"" + timeStamp, "" + leftPos,"" + rightPos, "" + leftVel, "" + rightVel, "" + voltage};
		//teleopDataLogger.writeLine(data);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		testMotors();
		//testDriveTrain();
		
	}
	
	//operatorControl() is called periodically in both teleop and test periodic
	public void operatorControl() {
		updateSmartDashboard();
		
		double currentTime = timer.get();
		SmartDashboard.putNumber("Current Time", currentTime);
		
		//Joystick - control tank drive
		DriveTrain.tankDrive(driveStick.getY(GenericHID.Hand.kLeft), driveStick.getY(GenericHID.Hand.kRight));
		//DriveTrain.tankDrive(0.8, 0.8);
		
		//double brightness = liftStick.getRawAxis(3);
		//System.out.println("Brightness " + brightness);
		
		//Back button - reset encoders and gyro
		if (driveStick.getBackButton()) {
			DriveTrain.resetEncoders();
			DriveTrain.gyro.reset();
			Elevator.encoder.reset();
		}		
		
		double netTrigger = driveStick.getTriggerAxis(GenericHID.Hand.kRight) - driveStick.getTriggerAxis(GenericHID.Hand.kLeft);
		
		//Dpad and bumpers - control rollers and pneumatics
	
		if (Math.abs(netTrigger) > 0.05) {
			Intake.roll(netTrigger);
		} else if (driveStick.getBumper(GenericHID.Hand.kRight)) {
            pickUpCube(); 
            System.out.println("Bumper right, pick up");
		} else if (driveStick.getBumper(GenericHID.Hand.kLeft)) {
			dropCube();
		} else if (liftStick.getRawButton(1)) {
			Intake.intake();
		}
		else {
			Intake.stopRollers();
		}
		
		
		//Actuating intake - toggle on side button (2) of big joystick
		if(liftStick.getRawButton(2) && intakeUp && !actuateButtonPressed) {
			Intake.actuateDown();
			actuateButtonPressed = true;
			intakeUp = false;
		} else if(liftStick.getRawButton(2) && !intakeUp && !actuateButtonPressed){
			Intake.actuateUp();
			intakeUp = true;
			actuateButtonPressed = true;
		} else if(!liftStick.getRawButton(2)) {
			actuateButtonPressed = false;
		}
		
		
		/*if (driveStick.getPOV(dPadChannel) > 325 || (driveStick.getPOV(dPadChannel) < 35 && driveStick.getPOV(dPadChannel) >= 0)) {
		intakeState = IntakeState.MANUAL;
		System.out.println("Dpad up");
		rollOut(); 
	} else if (driveStick.getPOV(dPadChannel) > 145 && driveStick.getPOV(dPadChannel) < 215) {
		intakeState = IntakeState.MANUAL;
		rollIn(); 
	}*/
		
		//Buttons - set Elevator lift to certain height - floor, switch, or scale
		if (((liftStick.getRawButton(11) || liftStick.getRawButton(12) || driveStick.getAButton()) && elevatorState != ElevatorState.FLOOR) && timer.get() < 145) {
			System.out.println("Move to floor button pressed");
			moveToFloor();
		} else if (((liftStick.getRawButton(9) || liftStick.getRawButton(10) || driveStick.getBButton()) && elevatorState != ElevatorState.SWITCH) && timer.get() < 145) {
			moveToSwitch();
		} else if (((liftStick.getRawButton(7) || liftStick.getRawButton(8) || driveStick.getYButton()) && elevatorState != ElevatorState.SCALE) && timer.get() < 145) {
			moveToScale();
		} else if (timer.get() > 145 && elevatorState != ElevatorState.FLOOR) {
			moveToFloor();
		}
		
		/*
		//Control Intake Angle
		if (liftStick.getRawButton(11) || liftStick.getRawButton(12)) {
			Intake.angleDown();
		} else if (liftStick.getRawButton(10) || liftStick.getRawButton(9)) {
			Intake.angleDiagonal();
		} else if (liftStick.getRawButton(8) || liftStick.getRawButton(7)) {
			Intake.angleUp();
		}*/
				
		//Operator Joystick - lift and lower Elevator
		
		double joystickValue = -liftStick.getRawAxis(1);
		//double netTrigger = driveStick.getTriggerAxis(GenericHID.Hand.kRight) - driveStick.getTriggerAxis(GenericHID.Hand.kLeft);
		//System.out.println("Net Trigger: " + netTrigger);
		if (Math.abs(joystickValue) >= 0.15 && timer.get() < 145) {
			triggerIntervalsCounted += 1;
	        previousHeight = Elevator.getCurrentPosition();
			Elevator.setTeleop(joystickValue, triggerIntervalsCounted);
			elevatorState = ElevatorState.MANUAL;
		} else if (driveStick.getXButton() && liftStick.getRawButton(2)) {
			elevatorState = ElevatorState.MANUAL;
			Elevator.motor.set(-0.9);
		} else if (elevatorState == ElevatorState.MANUAL && !Elevator.isAtFloor()) {
			triggerIntervalsCounted = 0;
			Elevator.maintainHeight(previousHeight);
		} else if (elevatorState == ElevatorState.MANUAL) {
			triggerIntervalsCounted = 0;
			Elevator.stop();
		} else {
			triggerIntervalsCounted = 0;
		}

		
		
		//Start button - deploy ramps at end of game
		if (timer.get() >= 120 && driveStick.getStartButton()) {
			deployRamps();
		}
		
		//*********************************************************************//
		
		//Check Elevator State
		if (Elevator.isAtFloor()) {
			Elevator.encoder.reset();
		}
		
		if (elevatorState == ElevatorState.FLOOR) {
			//double currentHeight = Elevator.getCurrentPosition();
			if (Elevator.isAtFloor()) { //Within desired range, stop elevating
				System.out.println("AT FLOOR, STOP");
				Elevator.stop();
				Elevator.encoder.reset();
				previousHeight = floorHeight;
				elevatorState = ElevatorState.MANUAL;
			} else {
				System.out.println("GOING TO FLOOR");
				Elevator.moveToHeight(floorHeight, distanceToLift, direction);
			}
		}
		
		if (elevatorState == ElevatorState.SWITCH) {
			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < switchHeight + elevatorTolerance && 
					currentHeight > switchHeight - elevatorTolerance) { //Within desired range, stop elevating
				//Elevator.stop();
				previousHeight = switchHeight;
				elevatorState = ElevatorState.MANUAL;
				System.out.println("STOP SWITCH");
			} else {
				Elevator.moveToHeight(switchHeight, distanceToLift, direction);
				System.out.println("SWITCH MOVE");
			}
		}
		
		if (elevatorState == ElevatorState.SCALE) {

			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < scaleHeight + elevatorTolerance && 
					currentHeight > scaleHeight - elevatorTolerance) { //Within desired range, stop elevating
				System.out.println("STOP SCALE");
			    //Elevator.stop();
				previousHeight = scaleHeight;
				elevatorState = ElevatorState.MANUAL;
			} else {
			    System.out.println("SCALE MOVE");
				Elevator.moveToHeight(scaleHeight, distanceToLift, direction);
			}
		}
		
		//*********************************************************************//
		
		/*
		//Check Intake State
		if (intakeState == IntakeState.INTAKING) {
			if (currentTime - intakeStartTime >= timeToRollIn) {
				//Stop based on time, or maybe a limit switch if implement
				Intake.stopRollers();
				intakeState = IntakeState.MANUAL;
			} else {
				Intake.intake();
			}
		}
		
		if (intakeState == IntakeState.OUTTAKING) {
			if (currentTime - intakeStartTime >= timeToRollOut) {
				Intake.stopRollers();
				intakeState = IntakeState.MANUAL;
			} else {
				Intake.outtake();
			}
		}*/
		
		//*********************************************************************//
		
	}
	
	//Left Bumper
	public void pickUpCube() {
		//intakeStartTime = timer.get();
		//intakeState = IntakeState.INTAKING;
		Intake.close();
		Intake.intake();
		//intakeOpen = false;
	}
	
	//Right Bumper
	public void dropCube() {
		intakeStartTime = timer.get();
		//intakeState = IntakeState.OUTTAKING;
		Intake.open();
		intakeOpen = true;
	}
	
	/*
	//Dpad up
	public void rollOut() {
		Intake.outtake();
	}
	
	//Dpad down
	public void rollIn() {
		Intake.intake();
	}*/
	
	//A Button
	public void moveToFloor() {
		//Elevator.setHeight(floorHeight);
		elevatorState = ElevatorState.FLOOR;
		direction = -1;
		previousHeight = Elevator.getCurrentPosition();
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - floorHeight);
	}
	
	//B Button
	public void moveToSwitch() {
		//Elevator.setHeight(switchHeight);
		elevatorState = ElevatorState.SWITCH;
		direction = (Elevator.getCurrentPosition() < switchHeight) ? 1 : -1;
	    previousHeight = Elevator.getCurrentPosition();
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - switchHeight);
	}
	
	//Y Button
	public void moveToScale() {
		//Elevator.setHeight(scaleHeight);
		elevatorState = ElevatorState.SCALE;
		direction = (Elevator.getCurrentPosition() < scaleHeight) ? 1 : -1;
	    previousHeight = Elevator.getCurrentPosition();
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - scaleHeight);
	}
	
	//Start Button
	public void deployRamps() {
		Ramps.deploy();
	}
	
	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Encoder position left", DriveTrain.leftTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder position right", DriveTrain.rightTalon.getSelectedSensorPosition(0));
		
		SmartDashboard.putNumber("Motor Output Left", DriveTrain.leftTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Motor Output Right", DriveTrain.rightTalon.getMotorOutputPercent());
		
		/*
		SmartDashboard.putNumber("Right MP position target", DriveTrain.rightTalon.getActiveTrajectoryPosition());
		SmartDashboard.putNumber("Left MP position target", DriveTrain.leftTalon.getActiveTrajectoryPosition());
		SmartDashboard.putNumber("Right MP velocity target", DriveTrain.leftTalon.getActiveTrajectoryVelocity());
		SmartDashboard.putNumber("Left MP velocity target", DriveTrain.rightTalon.getActiveTrajectoryVelocity());
		*/
		
		SmartDashboard.putNumber("Motor velocity left", DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Motor velocity right", DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		
		SmartDashboard.putNumber("Left closed loop error", DriveTrain.leftTalon.getClosedLoopError(0));
		SmartDashboard.putNumber("Right closed loop error", DriveTrain.rightTalon.getClosedLoopError(0));
		
		SmartDashboard.putNumber("Gyro Value", DriveTrain.getGyro());
		SmartDashboard.putNumber("Elevator Encoder Count", Elevator.encoder.get());
		SmartDashboard.putNumber("Elevator Current Height From Ground", Elevator.getCurrentPosition());
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
		if (Intake.solenoid.get() == DoubleSolenoid.Value.kReverse) {
		    intake = "Closed";
		}
		SmartDashboard.putString("Intake Pneumatic Status", intake);
	}
	
	public void testMotors() {
		if (driveStick.getAButton()) {
			Elevator.spark1.set(0.5);
		} else {
			Elevator.spark1.set(0);
		}
		
		if (driveStick.getBButton()) {
			Elevator.spark2.set(0.5);
		} else {
			Elevator.spark2.set(0);
		}
		
		if (driveStick.getYButton()) {
			DriveTrain.leftTalon.set(0.5);
		} else {
			DriveTrain.leftTalon.set(0.5);
		}
		
		if (driveStick.getXButton()) {
			DriveTrain.rightTalon.set(0.5);
		} else {
			DriveTrain.rightTalon.set(0.5);
		}
	}
	
	public void testDriveTrain() {
		
		double leftYStick = driveStick.getY(GenericHID.Hand.kLeft);
		if (leftYStick < -0.93)  {
			leftYStick = -1.0;
		}
		
		double motorOutputLeft = DriveTrain.leftTalon.getMotorOutputPercent();
		double motorOutputRight = DriveTrain.rightTalon.getMotorOutputPercent();
		
		sbL.append("LOut: ");
		sbR.append("ROut: ");
		sbL.append(motorOutputLeft);
		sbR.append(motorOutputRight);
		
		sbL.append("LSpd: ");
		sbR.append("RSpd: ");
		sbL.append(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		sbR.append(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		
		if (driveStick.getAButton()) {
			double targetVelocity  = leftYStick * convertVelocity(11);
			DriveTrain.leftTalon.set(ControlMode.Velocity, targetVelocity);
			DriveTrain.rightTalon.set(ControlMode.Velocity, targetVelocity);
			
			sbL.append("LErr:");
			sbR.append("RErr:");
			sbL.append(DriveTrain.leftTalon.getClosedLoopError(0));
			sbR.append(DriveTrain.rightTalon.getClosedLoopError(0));
			
			sbL.append("LTarg:");
			sbR.append("RTarg:");
			sbL.append(targetVelocity);
			sbR.append(targetVelocity);
		} else  {
			DriveTrain.leftTalon.set(ControlMode.PercentOutput, leftYStick);
			DriveTrain.rightTalon.set(ControlMode.PercentOutput, leftYStick);
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