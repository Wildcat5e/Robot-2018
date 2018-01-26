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
			rightIntakeSparkChannel = 5,
			elevatorSparkChannel = 6;
	
	//Solenoids
	public static int leftIntakeSolenoidForward = 0,
			leftIntakeSolenoidReverse = 1,
			rightIntakeSolenoidForward = 2,
			rightIntakeSolenoidReverse = 3;
	
	//Joysticks
	public static int driveStickChannel = 0,
			dPadChannel = 1;
	
	
	//Auto Starting Positions
	public static final String left = "left",
			middle = "middle",
			right = "right";
	
	//Sensors
	public static int elevatorEncoderSourceA = 0,
			elevatorEncoderSourceB = 1;
	
	//Driving Speeds in FPS
	public static double velocityFast = 12,
			velocityMedium = 9,
			velocitySlow = 6,
			velocityTurning = 5;
	
	//Time, distance, and other measurement
	public static double timeToRoll = 1.5, //seconds
		wheelRadius = 2.0, //inches
		ticksPerRevolution = 256, //encoder ticks
		distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
		rollersSpeed = 0.5,
		elevatorSpeed = 0.5; //From -1 to 1
	
	public static double convertInchesToTicks(double inches) {
		return (inches/(2 * Math.PI * wheelRadius)) * ticksPerRevolution;
	}
	
	public static double convertFPSToTicksPer100MS(double FPS) {
		double rpm = (60 * 12 * FPS)/(wheelRadius * 2 * Math.PI);
		return (rpm * ticksPerRevolution) / 600.0; // Rev/Min * Ticks/Rev * Min/100ms -> Ticks/100ms
	}
	
}
