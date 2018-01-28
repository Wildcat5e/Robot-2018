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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.Constants.*;


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
	
	boolean intakeOpen = true; 
	boolean intakeRolling = false;
	double intakeStartTime = 0;
	
	boolean elevatorMovingFloor = true;
	boolean elevatorMovingSwitch = true;
	boolean elevatorMovingScale = true;

	
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
		
		positionChooser.addObject("Left Starting Position", left);
		positionChooser.addObject("Middle Starting Position", middle);
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
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		switch (autoSelected) {
			case switchAuto:
				
				if(gameData.charAt(0) == 'L') {
					//Put left switch auto code here
					Autonomous.leftSwitchAuto(startingPosition);
				} else {
					//Put right switch auto code here
					Autonomous.rightSwitchAuto(startingPosition);
				}
				break;
			case scaleAuto:
				
				if(gameData.charAt(1) == 'L') {
					//Put left scale auto code here
					Autonomous.leftScaleAuto(startingPosition);
				} else {
					//Put right scale auto code here
					Autonomous.rightScaleAuto(startingPosition);
				}
				break;
			case baselineAuto:
				//Drive forward to cross baseline
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
	
	@Override
	public void teleopInit() {
		
		Intake.stopRollers();
		
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
		
		double currentTime = timer.get();
		
		SmartDashboard.putNumber("Current Time", currentTime);
		
		//Joystick - control tank drive
		DriveTrain.tankDrive(driveStick.getY(GenericHID.Hand.kLeft), driveStick.getY(GenericHID.Hand.kRight));
		
		//Bumpers - control intake pneumatics and rollers
		if (driveStick.getBumper(GenericHID.Hand.kRight) && intakeOpen && !intakeRolling) {
			bumperRight(); //Handle right bumper press
		} else if (driveStick.getBumper(GenericHID.Hand.kLeft) && !intakeOpen && !intakeRolling) {
			bumperLeft(); //Handle left bumper press
		}
		
		//dpad - control only rollers
		if (driveStick.getPOV(dPadChannel) > 325 || (driveStick.getPOV(dPadChannel) < 35 && driveStick.getPOV(dPadChannel) > 0)) {
			dPadUp(); //Handle dpad up press
		} else if (driveStick.getPOV(dPadChannel) > 145 && driveStick.getPOV(dPadChannel) < 215) {
			dPadDown(); //Handle dpad down press
		} else {
			Intake.stopRollers();
		}
		
		//Buttons - set elevator lift to certain height - floor, switch, or scale
		if (driveStick.getAButton()) {
			aButton();
		} else if (driveStick.getBButton()) {
			bButton();
		} else if (driveStick.getYButton()) {
			yButton();
		} 
		
		//Triggers - lift or lower elevator
		if (driveStick.getTriggerAxis(GenericHID.Hand.kLeft) > 0.5) {
			leftTrigger();
		} else if (driveStick.getTriggerAxis(GenericHID.Hand.kRight) > 0.5) {
			rightTrigger();
		}
		
		//Stop intaking if enough time has passed
		if (intakeRolling && currentTime - intakeStartTime > timeToRoll) {
			Intake.stopRollers();
			intakeRolling = false;
		}
		
		//Stop elevating if elevator has reached destination
		if (elevatorMovingFloor && 
				Elevator.getCurrentPosition() < floorPosition + elevatorTolerance && 
				Elevator.getCurrentPosition() > floorPosition - elevatorTolerance) {
			Elevator.stop();
			elevatorMovingFloor = false;
		}
		
		if (elevatorMovingSwitch && 
				Elevator.getCurrentPosition() < switchPosition + elevatorTolerance && 
				Elevator.getCurrentPosition() > switchPosition - elevatorTolerance) {
			Elevator.stop();
			elevatorMovingSwitch = false;
		}
		
		if (elevatorMovingScale && 
				Elevator.getCurrentPosition() < scalePosition + elevatorTolerance && 
				Elevator.getCurrentPosition() > scalePosition - elevatorTolerance) {
			Elevator.stop();
			elevatorMovingScale = false;
		}
		
		//Deploy ramps with start button
		if (timer.get() >= 120 && driveStick.getStartButton()) {
			startButton();
		}
		
	}
	
	//Called when driver presses right bumper
	public void bumperRight() {
		intakeStartTime = timer.get();
		Intake.close();
		intakeOpen = false;
		Intake.intakeCube();
		intakeRolling = true;
	}
	
	//Called when driver presses left bumper
	public void bumperLeft() {
		intakeStartTime = timer.get();
		Intake.open();
		intakeOpen = true;
		Intake.outtakeCube();
		intakeRolling = true;
	}
	
	//Called when driver presses dpad in general up direction
	public void dPadUp() {
		Intake.outtakeCube();
	}
	
	//Called when driver presses dpad in general down direction
	public void dPadDown() {
		Intake.intakeCube();
	}
	
	public void aButton() {
		elevatorMovingFloor = true;
		if (Elevator.getCurrentPosition() > floorPosition) {
			Elevator.lowerElevator();
		}
	}
	
	public void bButton() {
		elevatorMovingSwitch = true;
		if (Elevator.getCurrentPosition() > switchPosition) {
			Elevator.lowerElevator();
		} else {
			Elevator.liftElevator();
		}
	}
	
	public void yButton() {
		elevatorMovingScale = true;
		if (Elevator.getCurrentPosition() > scalePosition) {
			Elevator.lowerElevator();
		} else {
			Elevator.liftElevator();
		}
	}
	
	public void leftTrigger() {
		Elevator.lowerElevator();
	}
	
	public void rightTrigger() {
		Elevator.liftElevator();
	}
	// pressing the start button the robot deploys ramps (we win);
	public void startButton() {
		Ramps.deploy();
	}
	
}

