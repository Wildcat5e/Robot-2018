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
	
	//Rollers
	public static final double timeToRollIn = 1.5, //seconds
		timeToRollOut = 1.5; //seconds
		
	
	//Drive Train
	public static final double wheelRadius = 2.937, //inches
			ticksPerRevolution = 2048 * 4, //encoder ticks, multiplied by 4 because quadrature encoders do 4 pulses per count
			distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
			rollersSpeed = 0.5,//[-1, 1]
			turningTolerance = 2.5, //Degrees
			deadband = 0.04, //-1 to 1
			rampRate = 0.5, //Seconds to ramp from 0 to full
			stallCurrent = 30, //Amps
			maxTicksPer100ms = 5600,//This is the max speed in native units per 100 ms of the motors (percent output 100%)
			maxError = 400,
			minimumSpeed = 900,//ticks per 100 ms
			minimumTurningSpeed = 950,
			angleP = 70;
	
	//Driving Speeds in Feet Per Second (FPS)
		public static final double velocityMax = getFPS(maxTicksPer100ms),
				velocityFast = 9,
				velocityMedium = 6,
				velocitySlow = 3,
				velocityTurning = 3;
	
	//Elevator Constants
	public static final double elevatorSpeedMax = 0.7,
			verticalInchesPerTick = 0.1,
			elevatorTolerance = 0.5, //Inches
			maximumHeight = 90,
			floorHeight = 7.0,
			switchHeight = 25.0,
			scaleHeight = 82.0;
	
	//PID for DriveTrain
	public static double kP_R = 0.0004,//-(1023 * 0.1)/maxError,
			kP_L  = 0.0035,//(1023 * 0.05)/maxError,
			kD = 0,//kP * 10,//1023.0/maxError,
			kI = 0,//1.023/maxError,
			kF = 0.935  * 1023.0/maxTicksPer100ms;
	
	public static double convertInchesToTicks(double inches) {
		return (inches/(2 * Math.PI * wheelRadius)) * ticksPerRevolution;
	}
	
	public static double convertVelocity(double FPS) {
		double rpm = (60 * 12 * FPS)/(wheelRadius * 2 * Math.PI);
		return (rpm * ticksPerRevolution) / 600.0; // Rev/Min * Ticks/Rev * Min/100ms -> Ticks/100ms
	}
	
	public static double getFPS(double ticksPer100ms) {
		return (ticksPer100ms * 2 * wheelRadius * Math.PI * 600)/(60 * 12 * ticksPerRevolution);
	}
	
}
