/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

public class Constants {

	//Motor Controllers
	public static int leftTalonChannel = 15,
			rightTalonChannel = 1,
			leftVictorChannel = 2,
			rightVictorChannel = 3,
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

	
	//Speed, time, and other constants
	public static double timeToRoll = 1.5,
		autoForwardSpeedRPM = 0.4,
		wheelRadius = 2.0,
		pulsesPerRotation = 256,
		distancePerPulse = (wheelRadius * 2.0 * Math.PI)/pulsesPerRotation,
		rollersSpeed = 1.0,
		talonAcceleration = 100;
	
	
}
