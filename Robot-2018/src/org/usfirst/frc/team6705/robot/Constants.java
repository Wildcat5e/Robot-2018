/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

public class Constants {

	//Motor Controllers
	public static int frontLeftMotorChannel = 0,
			frontRightMotorChannel = 1,
			backLeftMotorChannel = 2,
			backRightMotorChannel = 3,
			leftIntakeSparkChannel = 4,
			rightIntakeSparkChannel = 5;
	
	//Solenoids
	public static int leftIntakeSolenoidForward = 0,
			leftIntakeSolenoidReverse = 1,
			rightIntakeSolenoidForward = 2,
			rightIntakeSolenoidReverse = 3;
	
	//Joysticks
	public static int driveStickLeftYAxis = 1,
			driveStickRightYAxis = 5;
	
	
	
	//Sensors
	public static int driveEncoderLeftChannelA = 0,
			driveEncoderLeftChannelB = 1,
			driveEncoderRightChannelA = 2,
			driveEncoderRightChannelB = 3;
	
	//Speed, time, and other constants
	public static int rollersSpeed = 1;
			
	
	public static double timeToRoll = 1.5,
		autoForwardSpeed = 0.4,
		wheelRadius = 2.0,
		pulsesPerRotation = 100,
		distancePerPulse = (wheelRadius * 2.0 * Math.PI)/pulsesPerRotation;
	
	
}
