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
	public static int driveStickChannel = 0,
			driveStickLeftYAxis = 1,
			driveStickRightYAxis = 5,
			dPadChannel = 1;
	
	
	//Auto Starting Positions
	public static final String left = "left",
			middle = "middle",
			right = "right";
	
	//Sensors

	
	//Speed, time, measurement, and other constants
	public static double timeToRoll = 1.5, //seconds
		wheelRadius = 2.0, //inches
		autoForwardSpeedFPS = 9, //feet per second
		autoForwardSpeedRPM = (60 *autoForwardSpeedFPS * 12)/(wheelRadius * 2 * Math.PI), //rev per minute
		ticksPerRevolution = 256, //encoder ticks
		distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
		rollersSpeed = 0.5; //From -1 to 1
	
	
}
