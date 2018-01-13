/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up                                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
	private SendableChooser<String> chooser = new SendableChooser<>();
	
	Spark frontLeftMotor = new Spark(Constants.frontLeftMotor);
	Spark frontRightMotor = new Spark(Constants.frontRightMotor);
	Spark backLeftMotor = new Spark (Constants.backLeftMotor);
	Spark backRightMotor = new Spark (Constants.backRightMotor);
	
	SpeedControllerGroup left = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
	SpeedControllerGroup right = new SpeedControllerGroup(frontRightMotor, backRightMotor);
	
	DifferentialDrive robotDrive = new DifferentialDrive(left, right);
	
	Joystick driveStick = new Joystick(1);
	
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Power Cube on Switch", switchAuto);
		chooser.addObject("Power Cube on Scale", scaleAuto);
		chooser.addObject("Cross Baseline Only", baselineAuto);
		SmartDashboard.putData("Auto choices", chooser);
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
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();

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
		
		robotDrive.tankDrive(driveStick.getRawAxis(Constants.driveStickLeftYAxis), driveStick.getRawAxis(Constants.driveStickRightYAxis), true);
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		robotDrive.tankDrive(driveStick.getRawAxis(Constants.driveStickLeftYAxis), driveStick.getRawAxis(Constants.driveStickRightYAxis), true);

		
	}
}

