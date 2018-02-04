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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Elevator.ElevatorState;
import org.usfirst.frc.team6705.robot.Intake.IntakeState;


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
	private static final String scaleAuto = "scale";
	private static final String baselineAuto = "baseline";
	private static final String bestSimple = "bestSimple";
	private String autoSelected;
	
	private String startingPosition;
	
	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> positionChooser = new SendableChooser<>();
	
	boolean intakeOpen = false; 
	IntakeState intakeState = IntakeState.MANUAL;
	double intakeStartTime = 0;
	
	double distanceToLift = 0;
	ElevatorState elevatorState = ElevatorState.MANUAL;
	
	Timer timer = new Timer();
	
	XboxController driveStick = new XboxController(driveStickChannel);
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser.addDefault("Power Cube on Switch", switchAuto);
		autoChooser.addObject("Power Cube on Scale", scaleAuto);
		autoChooser.addObject("Best Simple Scoring Method", bestSimple);
		autoChooser.addObject("Cross Baseline Only", baselineAuto);
		SmartDashboard.putData("Auto choices", autoChooser);
		
		positionChooser.addDefault("Middle Starting Position", middle);
		positionChooser.addObject("Left Starting Position", left);
		positionChooser.addObject("Right Starting Position", right);
		SmartDashboard.putData("Starting position", positionChooser);
		
		DriveTrain.setup();
		Elevator.setup();
		Intake.setup();
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
		startingPosition = positionChooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();

		timer.start();
		
		switch (autoSelected) {
		case switchAuto:
			
			if(gameData.charAt(0) == 'L') {
				Autonomous.leftSwitchAuto(startingPosition);
			} else {
				Autonomous.rightSwitchAuto(startingPosition);
			}
			break;
		case scaleAuto:
			
			if(gameData.charAt(1) == 'L') {
				Autonomous.leftScaleAuto(startingPosition);
			} else {
				Autonomous.rightScaleAuto(startingPosition);
			}
			break;
		case baselineAuto:
			Autonomous.baselineAuto();
			break;
		case bestSimple:
			switch(startingPosition) {
			case left:
				if(gameData.charAt(1) == 'L') {
					Autonomous.leftScaleAuto(startingPosition);
				} else if(gameData.charAt(0) == 'L'){
					Autonomous.leftSwitchAuto(startingPosition);
				} else {
					Autonomous.baselineAuto();
				}
				break;
			case middle:
				if(gameData.charAt(0) == 'L') {
					Autonomous.leftSwitchAuto(startingPosition);
				} else {
					Autonomous.rightSwitchAuto(startingPosition);
				}
				break;
			case right:
				if(gameData.charAt(1) == 'R') {
					Autonomous.rightScaleAuto(startingPosition);
				} else if(gameData.charAt(0) == 'R') {
					Autonomous.rightSwitchAuto(startingPosition);
				} else {
					Autonomous.baselineAuto();
				}
				break;
			}
			break;
		default:
			// Put default auto code here
			break;
	}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		updateSmartDashboard();

	}
	
	@Override
	public void teleopInit() {
		
		Intake.stopRollers();
		
		if (Intake.leftSolenoid.get() == DoubleSolenoid.Value.kForward) {
			intakeOpen = true;
		} else if (Intake.leftSolenoid.get() == DoubleSolenoid.Value.kReverse) {
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
		
		//Joystick - control tank drive
		DriveTrain.tankDrive(driveStick.getY(GenericHID.Hand.kLeft), driveStick.getY(GenericHID.Hand.kRight));
		
		//Bumpers - control intake pneumatics and rollers
		if (driveStick.getBumper(GenericHID.Hand.kRight) && !intakeOpen) {
			dropCube(); 
		} else if (driveStick.getBumper(GenericHID.Hand.kLeft) && intakeOpen) {
			pickUpCube();
		}
		
		//Dpad - control only rollers
		if (driveStick.getPOV(dPadChannel) > 325 || (driveStick.getPOV(dPadChannel) < 35 && driveStick.getPOV(dPadChannel) >= 0)) {
			intakeState = IntakeState.MANUAL;
			rollIn(); 
		} else if (driveStick.getPOV(dPadChannel) > 145 && driveStick.getPOV(dPadChannel) < 215) {
			intakeState = IntakeState.MANUAL;
			rollOut(); 
		} else if (intakeState == IntakeState.MANUAL) {
			Intake.stopRollers();
		}
		
		//Buttons - set elevator lift to certain height - floor, switch, or scale
		if (driveStick.getAButton()) {
			moveToFloor();
		} else if (driveStick.getBButton()) {
			moveToSwitch();
		} else if (driveStick.getYButton()) {
			moveToScale();
		} 
		
		//Triggers - lift or lower elevator
		if (driveStick.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.05) {
			moveElevatorDown(driveStick.getTriggerAxis(GenericHID.Hand.kLeft));
			elevatorState = ElevatorState.MANUAL;
		} else if (driveStick.getTriggerAxis(GenericHID.Hand.kRight) >= 0.05) {
			moveElevatorUp(driveStick.getTriggerAxis(GenericHID.Hand.kLeft));
			elevatorState = ElevatorState.MANUAL;
		} else if (elevatorState == ElevatorState.MANUAL) {
			Elevator.stop();
		}
		
		//Start button - deploy ramps at end of game
		if (timer.get() >= 120 && driveStick.getStartButton()) {
			deployRamps();
		}
		
		//*********************************************************************//
		
		//Check Intake State
		if (intakeState == IntakeState.INTAKING) {
			if (currentTime - intakeStartTime >= timeToRoll) {
				//Stop based on time, or maybe a limit switch if implement
				Intake.stopRollers();
				intakeState = IntakeState.MANUAL;
			} else {
				Intake.intake();
			}
		}
		
		if (intakeState == IntakeState.OUTTAKING) {
			if (currentTime - intakeStartTime >= timeToRoll) {
				Intake.stopRollers();
				intakeState = IntakeState.MANUAL;
			} else {
				Intake.outtake();
			}
		}
		
		//*********************************************************************//
		
		//Check Elevator State
		if (elevatorState == ElevatorState.FLOOR) {
			
			double currentHeight = Elevator.getCurrentPosition();
			if (currentHeight < floorHeight + elevatorTolerance && 
					currentHeight > floorHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(floorHeight, currentHeight, distanceToLift);
			}
		}
		
		if (elevatorState == ElevatorState.SWITCH) {
			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < switchHeight + elevatorTolerance && 
					currentHeight > switchHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(switchHeight, currentHeight, distanceToLift);
			}
		}
		
		if (elevatorState == ElevatorState.SCALE) {
			double currentHeight = Elevator.getCurrentPosition();
			
			if (currentHeight < scaleHeight + elevatorTolerance && 
					currentHeight > scaleHeight - elevatorTolerance) { //Within desired range, stop elevating
				Elevator.stop();
				elevatorState = ElevatorState.MANUAL;
			} else {
				Elevator.moveToHeight(scaleHeight, currentHeight, distanceToLift);
			}
		}
		
	}
	
	//Left Bumper
	public void pickUpCube() {
		intakeStartTime = timer.get();
		intakeState = IntakeState.INTAKING;
		Intake.close();
		intakeOpen = false;
	}
	
	//Right Bumper
	public void dropCube() {
		intakeStartTime = timer.get();
		intakeState = IntakeState.OUTTAKING;
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
		elevatorState = ElevatorState.FLOOR;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - floorHeight);
	}
	
	//B Button
	public void moveToSwitch() {
		elevatorState = ElevatorState.SWITCH;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - switchHeight);
	}
	
	//Y Button
	public void moveToScale() {
		elevatorState = ElevatorState.SCALE;
		distanceToLift = Math.abs(Elevator.getCurrentPosition() - scaleHeight);
	}
	
	//Left Trigger
	public void moveElevatorDown(double speed) {
		Elevator.set(-speed);
	}
	
	//Right Trigger
	public void moveElevatorUp(double speed) {
		Elevator.set(speed);
	}
	
	//Start Button
	public void deployRamps() {
		Ramps.deploy();
	}
	
	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Encoder Count Left", DriveTrain.leftTalon.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder Count Right", DriveTrain.rightTalon.getSelectedSensorPosition(0));
	}
	
}

