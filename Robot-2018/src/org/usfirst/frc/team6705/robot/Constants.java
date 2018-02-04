/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

public class Constants {

	//Motor Controllers
	public static final int leftTalonChannel = 1,
			rightTalonChannel = 0,
			leftVictorChannel = 1,
			rightVictorChannel = 0,
			leftIntakeSparkChannel = 4,
			rightIntakeSparkChannel = 5,
			elevatorSparkChannel = 6;
	
	//Solenoids
	public static final int leftIntakeSolenoidForward = 0,
			leftIntakeSolenoidReverse = 1,
			rightIntakeSolenoidForward = 2,
			rightIntakeSolenoidReverse = 3,
			leftRampSolenoidOpen = 4,
			leftRampSolenoidClosed = 5,
			rightRampSolenoidOpen = 6,
			rightRampSolenoidClosed = 7;
	
	//Joysticks
	public static final int driveStickChannel = 0,
			dPadChannel = 0;
	
	
	//Auto Starting Positions
	public static final String left = "left",
			middle = "middle",
			right = "right";
	
	//Sensors
	public static final int elevatorEncoderSourceA = 0,
			elevatorEncoderSourceB = 1;
	
	//Driving Speeds in FPS
	public static final double velocityFast = 12,
			velocityMedium = 9,
			velocitySlow = 6,
			velocityTurning = 5;
	
	//Rollers and drive train
	public static final double timeToRoll = 1.5, //seconds
		wheelRadius = 3.0, //inches
		ticksPerRevolution = 256, //encoder ticks
		distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
		rollersSpeed = 0.5,//[-1, 1]
		turningTolerance = 2,
		deadband = 0.02; //Degrees
		
	
	//Elevator Constants
	public static final double elevatorSpeedMax = 0.7,
			verticalInchesPerTick = 0.1,
			elevatorTolerance = 0.5, //Inches
			maximumHeight = 90,
			floorHeight = 7.0,
			switchHeight = 25.0,
			scaleHeight = 82.0;
	
	//PID
	/*public static double kP = 0.1,
			kI = 0.001,
			kD = 0,
			kF = 1;*/
	
	public static double convertInchesToTicks(double inches) {
		return (inches/(2 * Math.PI * wheelRadius)) * ticksPerRevolution;
	}
	
	public static double convertFPSToTicksPer100MS(double FPS) {
		double rpm = (60 * 12 * FPS)/(wheelRadius * 2 * Math.PI);
		return (rpm * ticksPerRevolution) / 600.0; // Rev/Min * Ticks/Rev * Min/100ms -> Ticks/100ms
	}
	
}
