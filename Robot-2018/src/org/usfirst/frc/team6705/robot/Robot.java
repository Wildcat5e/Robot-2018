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

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
	
	private static final String switchAuto = "Place Power Cube on Switch";
	private static final String scaleAuto = "Place Power Cube on Scale";
	private static final String baselineAuto = "Cross Baseline Only";
	private String autoSelected;
	
	private static final String leftPosition = "Left Starting Position";
	private static final String middlePosition = "Middle Starting Position";
	private static final String rightPosition = "Right Starting Position";
	
	private SendableChooser<String> autoChooser = new SendableChooser<>();
	private SendableChooser<String> positionChooser = new SendableChooser<>();
	
	Spark frontLeftMotor = new Spark(frontLeftMotorChannel);
	Spark frontRightMotor = new Spark(frontRightMotorChannel);
	Spark backLeftMotor = new Spark (backLeftMotorChannel);
	Spark backRightMotor = new Spark (backRightMotorChannel);
	
	boolean intakeOpen = true; 
	boolean intakeRolling = false;
	double intakeStartTime = 0;
	
	Timer timer = new Timer();
	
	Encoder driveTrainEncoderLeft = new Encoder(driveEncoderLeftChannelA, driveEncoderLeftChannelB, false, CounterBase.EncodingType.k4X);
	Encoder driveTrainEncoderRight = new Encoder(driveEncoderRightChannelA, driveEncoderRightChannelB, false, CounterBase.EncodingType.k4X);

	
	SpeedControllerGroup left = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
	SpeedControllerGroup right = new SpeedControllerGroup(frontRightMotor, backRightMotor);
	
	DifferentialDrive robotDrive = new DifferentialDrive(left, right);
	
	XboxController driveStick = new XboxController(1);
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser.addDefault("Power Cube on Switch", switchAuto);
		autoChooser.addObject("Power Cube on Scale", scaleAuto);
		autoChooser.addObject("Cross Baseline Only", baselineAuto);
		SmartDashboard.putData("Auto choices", autoChooser);
		
		positionChooser.addObject("Left Starting Position", leftPosition);
		positionChooser.addObject("Middle Starting Position", middlePosition);
		positionChooser.addObject("Right Starting Position", rightPosition);
		SmartDashboard.putData("Starting position", positionChooser);
		
		driveTrainEncoderLeft.setDistancePerPulse(distancePerPulse);
		driveTrainEncoderRight.setDistancePerPulse(distancePerPulse);

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
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		timer.start();
		driveTrainEncoderLeft.reset();
		driveTrainEncoderRight.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		double currentDistanceLeft = driveTrainEncoderLeft.getDistance();
		double currentDistanceRight = driveTrainEncoderRight.getDistance();
		
		switch (autoSelected) {
			case switchAuto:
				
				if(gameData.charAt(0) == 'L') {
					//Put left switch auto code here
				} else {
					//Put right switch auto code here
				}
				break;
			case scaleAuto:
				
				if(gameData.charAt(1) == 'L') {
					//Put left scale auto code here
				} else {
					//Put right scale auto code here
				}
				break;
			case baselineAuto:
				//Drive forward to cross baseline
				if (currentDistanceRight <= 60) {
					Autonomous.driveForward(robotDrive, autoForwardSpeed);
				} else {
					Autonomous.stop(robotDrive);
				}
				break;
			default:
				// Put default auto code here
				break;
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
	
	public void operatorControl() {
		
		double currentTime = timer.get();
		
		robotDrive.tankDrive(driveStick.getRawAxis(driveStickLeftYAxis), driveStick.getRawAxis(driveStickRightYAxis), true);

		if (driveStick.getBumper(GenericHID.Hand.kRight) && intakeOpen && !intakeRolling) {
			//Pressed right bumper, close intake
			bumperRight();
			
		} else if (driveStick.getBumper(GenericHID.Hand.kLeft) && !intakeOpen && !intakeRolling) {
			//Pressed left bump, close outtake
			bumperLeft();
		}
		
		//Stop intaking
		if (intakeRolling && currentTime - intakeStartTime > timeToRoll) {
			Intake.stopRollers();
			intakeRolling = false;
		}
		
	}
	
	public void bumperRight() {
		intakeStartTime = timer.get();
		Intake.close();
		intakeOpen = false;
		Intake.intakeCube();
		intakeRolling = true;
	}
	
	public void bumperLeft() {
		intakeStartTime = timer.get();
		Intake.open();
		intakeOpen = true;
		Intake.outtakeCube();
		intakeRolling = true;
	}
	
	
}

