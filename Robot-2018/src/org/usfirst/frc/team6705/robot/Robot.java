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

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Elevator;
import org.usfirst.frc.team6705.robot.Elevator.ElevatorState;
import org.usfirst.frc.team6705.robot.Intake.IntakeState;

import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project
 */

public class Robot extends IterativeRobot {
	private String gameData;
	
	private static final String switchAuto = "switch";
	private static final String twoCubeAuto = "twoCube";
	private static final String baselineAuto = "baseline";
	private static final String bestSimple = "bestSimple";
	private static final String test = "test";
	private static final String basic = "basic";
	private static final String stall = "stall";
	private String autoSelected;
	
	private String startingPosition;
	
	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> positionChooser = new SendableChooser<>();
	
	public static Autonomous auto = new Autonomous();
	//public static Elevator Elevator = new Elevator();
	
	boolean intakeOpen = false; 
	IntakeState intakeState = IntakeState.MANUAL;
	double intakeStartTime = 0;
	
	double distanceToLift = 0;
	double previousHeight = floorHeight;
	ElevatorState elevatorState = ElevatorState.MANUAL;
	
	public static Timer timer = new Timer();
	
	XboxController driveStick = new XboxController(driveStickChannel);
	//Compressor compressor = new Compressor();
	StringBuilder sbL = new StringBuilder();
	StringBuilder sbR  = new StringBuilder();
	
	int loops = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser.addDefault("Power Cube on Switch", switchAuto);
		autoChooser.addObject("Power Cube on Scale", twoCubeAuto);
		autoChooser.addObject("Best Simple Scoring Method", bestSimple);
		autoChooser.addObject("Cross Baseline Only", baselineAuto);
		autoChooser.addObject("Test Auto - Drive, Turn, Drive", test);
		autoChooser.addObject("Super Basic Test Auto", basic);
		autoChooser.addObject("Stalling Test Auto", stall);
		SmartDashboard.putData("Auto choices", autoChooser);
		
		positionChooser.addDefault("Left Starting Position", left);
		positionChooser.addObject("Middle Starting Position", middle);
		positionChooser.addObject("Right Starting Position", right);
		SmartDashboard.putData("Starting position", positionChooser);
		SmartDashboard.putNumber("Stall Amps", Constants.stallCurrent);
		
		Constants.setup();
		DriveTrain.setup();
		Elevator.setup();
		Intake.setup();
		
		//CameraServer.getInstance().startAutomaticCapture();
		
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
		//autoSelected = test;
		startingPosition = positionChooser.getSelected();
		System.out.print("Autonomous Init");
		System.out.print(autoSelected);
		
		SmartDashboard.putNumber("Auto State", 0);
		
		gameData = "LLL";
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		timer.start();
		//DriveTrain.gyro.setPIDSourceType(PIDSourceType.kDisplacement);//Remove this line if it causes issues
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		updateSmartDashboard();
		
		switch (autoSelected) {
		case switchAuto:
			if (gameData.charAt(0) == 'L') {
				auto.switchAuto(startingPosition, 1, (gameData.charAt(1) == 'L') ? 1 : -1); //Note: side 1 means left, side -1 means right
			} else {
				auto.switchAuto(startingPosition, -1, (gameData.charAt(1) == 'L') ? 1 : -1);
			}
			break;
			
		case baselineAuto:
			System.out.print("Baseline Auto Periodic");
			auto.baselineAuto();
			break;
			
		case twoCubeAuto:
			if (gameData.charAt(1) == 'L') {
				auto.twoCubeAuto(startingPosition, 1, (gameData.charAt(0) == 'L') ? 1 : -1);
			} else {
				auto.twoCubeAuto(startingPosition, -1, (gameData.charAt(0) == 'L') ? 1 : -1);
			}
			break;
			
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
			break;
		case test:
			System.out.println("Running test auto");
			auto.testAuto();
			break;
		case basic:
			auto.basicAuto();
			break;
		case stall:
		    auto.testStallAuto();
		    break;
		}
		

	}
	
	@Override
	public void teleopInit() {
		
		DriveTrain.stop();
		Intake.stopRollers();
		Elevator.stop();
		
		DriveTrain.leftTalon.configClosedloopRamp(rampRateTeleop, 0);
		DriveTrain.rightTalon.configClosedloopRamp(rampRateTeleop, 0);
		
		//If the intake is open due to whatever happened in auto, set intakeOpen to true
		if (Intake.solenoid.get() == DoubleSolenoid.Value.kForward) {
			intakeOpen = true;
		} else if (Intake.solenoid.get() == DoubleSolenoid.Value.kReverse) {
			intakeOpen = false;
		}
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		operatorControl();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		operatorControl();
		
	}
	
	//operatorControl() is called periodically in both teleop and test periodic
	public void operatorControl() {
		updateSmartDashboard();
		
		double currentTime = timer.get();
		SmartDashboard.putNumber("Current Time", currentTime);
		
		/*
		double leftYStick = driveStick.getY(GenericHID.Hand.kLeft);
		if (leftYStick < -0.93)  {
			leftYStick = -1.0;
		}
		
		double motorOutputLeft = DriveTrain.leftTalon.getMotorOutputPercent();
		double motorOutputRight = DriveTrain.rightTalon.getMotorOutputPercent();
		
		sbL.append("LOut:");
		sbR.append("ROut:");
		sbL.append(motorOutputLeft);
		sbR.append(motorOutputRight);
		
		sbL.append("LSpd:");
		sbR.append("RSpd");
		sbL.append(DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		sbR.append(DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		
		if (driveStick.getAButton()) {
			double targetVelocity  = leftYStick * 350 * 1024 / 600;
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
		sbR.setLength(0);*/
		
		
		
		//Joystick - control tank drive
		DriveTrain.tankDrive(driveStick.getY(GenericHID.Hand.kLeft), driveStick.getY(GenericHID.Hand.kRight));
		//DriveTrain.tankDrive(0.8, 0.8);
		
		//Back button - reset encoders and gyro
		if (driveStick.getBackButton()) {
			DriveTrain.resetEncoders();
			DriveTrain.gyro.reset();
			Elevator.encoder.reset();
		}
		
		//Bumpers - control intake pneumatics and rollers
		
		
		//Dpad - control only rollers
		if (driveStick.getPOV(dPadChannel) > 325 || (driveStick.getPOV(dPadChannel) < 35 && driveStick.getPOV(dPadChannel) >= 0)) {
			intakeState = IntakeState.MANUAL;
			System.out.println("Dpad up");
			rollOut(); 
		} else if (driveStick.getPOV(dPadChannel) > 145 && driveStick.getPOV(dPadChannel) < 215) {
			intakeState = IntakeState.MANUAL;
			rollIn(); 
		} else if (driveStick.getBumper(GenericHID.Hand.kRight)) {
            pickUpCube(); 
            System.out.println("Bumper right, pick up");
       } else if (driveStick.getBumper(GenericHID.Hand.kLeft)) {
           dropCube();
       } else {
           Intake.stopRollers();
       }
		
		//Buttons - set Elevator lift to certain height - floor, switch, or scale
		if (driveStick.getAButton()) {
			moveToFloor();
		} else if (driveStick.getBButton()) {
			moveToSwitch();
		} else if (driveStick.getYButton()) {
			moveToScale();
		} 
				
		//Triggers - lift and lower Elevator

		double netTrigger = driveStick.getTriggerAxis(GenericHID.Hand.kRight) - driveStick.getTriggerAxis(GenericHID.Hand.kLeft);
		
		if (Math.abs(netTrigger) >= 0.1) {
	        previousHeight = Elevator.getCurrentPosition();
			moveElevator(netTrigger);
			elevatorState = ElevatorState.MANUAL;
		} else if (elevatorState == ElevatorState.MANUAL && Elevator.getCurrentPosition() > floorHeight + elevatorTolerance) {
			Elevator.maintainHeight(previousHeight);
		}
		
		/*
		//Triggers - lift or lower Elevator
		if (driveStick.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.05) {
			moveElevatorDown(driveStick.getTriggerAxis(GenericHID.Hand.kLeft));
			previousHeight = Elevator.getCurrentPosition();
			ElevatorState = ElevatorState.MANUAL;
		} else if (driveStick.getTriggerAxis(GenericHID.Hand.kRight) >= 0.05) {
			moveElevatorUp(driveStick.getTriggerAxis(GenericHID.Hand.kRight));
			previousHeight = Elevator.getCurrentPosition();
			ElevatorState = ElevatorState.MANUAL;
		} else if (ElevatorState == ElevatorState.MANUAL) {
			Elevator.maintainHeight(previousHeight);
		}
		
		//Start button - deploy ramps at end of game
		if (timer.get() >= 120 && driveStick.getStartButton()) {
			deployRamps();
		}*/
		
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
		
		
		//Check Elevator State
		if (elevatorState == ElevatorState.FLOOR) {
			
			//if (Elevator.pid.onTarget()) {
				//elevatorState = ElevatorState.MANUAL;
			//}
			
			double currentHeight = Elevator.getCurrentPosition();
			if (currentHeight < floorHeight + elevatorTolerance && 
					currentHeight > floorHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				previousHeight = floorHeight;
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(floorHeight, currentHeight, distanceToLift);
			}
		}
		
		if (elevatorState == ElevatorState.SWITCH) {
			//if (Elevator.pid.onTarget()) {
			//	elevatorState = ElevatorState.MANUAL;
			//}
			
			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < switchHeight + elevatorTolerance && 
					currentHeight > switchHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				previousHeight = switchHeight;
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(switchHeight, currentHeight, distanceToLift);
			}
		}
		
		if (elevatorState == ElevatorState.SCALE) {
			//if (Elevator.pid.onTarget()) {
				//elevatorState = ElevatorState.MANUAL;
			//}
			
			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < scaleHeight + elevatorTolerance && 
					currentHeight > scaleHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				previousHeight = scaleHeight;
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(scaleHeight, currentHeight, distanceToLift);
			}
		}
		
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
	
	//Dpad up
	public void rollOut() {
		Intake.outtake();
	}
	
	//Dpad down
	public void rollIn() {
		Intake.intake();
	}
	
	//A Button
	public void moveToFloor() {
		Elevator.setHeight(floorHeight);
		elevatorState = ElevatorState.FLOOR;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - floorHeight);
	}
	
	//B Button
	public void moveToSwitch() {
		Elevator.setHeight(switchHeight);
		elevatorState = ElevatorState.SWITCH;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - switchHeight);
	}
	
	//Y Button
	public void moveToScale() {
		Elevator.setHeight(scaleHeight);
		elevatorState = ElevatorState.SCALE;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - scaleHeight);
	}
	
	/*
	//Left Trigger
	public void moveElevatorDown(double speed) {
		//Elevator.set(-speed);
		Elevator.setHeight(-speed * );
	}
	
	//Right Trigger
	public void moveElevatorUp(double speed) {
		//Elevator.set(speed);
	}*/
	
	//Both Triggers (Left is negative/down, right is positive/up)
	public void moveElevator(double speed) {
		//Elevator.setHeight(previousHeight + (speed * (maximumHeight - floorHeight)));
	    Elevator.set(speed);
	}
	
	//Start Button
	public void deployRamps() {
		Ramps.deploy();
	}
	
	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Encoder Count Left", DriveTrain.leftTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder Count Right", DriveTrain.rightTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Motor Speed Left", DriveTrain.leftTalon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Motor Speed Right", DriveTrain.rightTalon.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Gyro Value", DriveTrain.getGyro());
		SmartDashboard.putNumber("Left Talon Current", DriveTrain.leftTalon.getOutputCurrent());
		SmartDashboard.putNumber("Right Talon Current", DriveTrain.rightTalon.getOutputCurrent());
		SmartDashboard.putNumber("Elevator Encoder Count", Elevator.encoder.get());
		SmartDashboard.putNumber("Elevator Current Height From Ground", Elevator.getCurrentPosition());
	}
	
}