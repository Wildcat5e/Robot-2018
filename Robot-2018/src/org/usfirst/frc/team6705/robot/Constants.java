/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.Preferences;

//import edu.wpi.first.wpilibj.Preferences;

public class Constants {
	
	//Motor Controllers
	public static final int
			leftTalonChannel = 0, 
			rightTalonChannel = 1,
			leftVictorChannel = 0,
			rightVictorChannel = 1,
			leftIntakeSparkChannel = 2,
			rightIntakeSparkChannel = 4,
			elevatorSpark1 = 0,
			elevatorSpark2 = 1;
	
	//Solenoids
	public static final int intakeSolenoidA = 0,
			intakeSolenoidB = 1,
			leftRampSolenoidA = 2,
			leftRampSolenoidB = 3,
			rightRampSolenoidA = 4,
			rightRampSolenoidB = 5;
	
	//Joysticks
	public static final int driveStickChannel = 0,
			dPadChannel = 0,
			liftStickChannel = 1;
	
	
	//Auto Starting Positions
	public static final String left = "left",
			middle = "middle",
			right = "right";
	
	//Sensors
	public static final int elevatorEncoderSourceA = 0,
			elevatorEncoderSourceB = 1,
			elevatorLimitSwitchDIO = 2;
	
	//Rollers
	public static final double timeToRollIn = 0.5, //seconds
		timeToRollOut = 1, //seconds
		maxRollersSpeed = 0.7,
		rollersSpeedAuto = 0.7;//[-1, 1]
		
	
	//Drive Train
	public static final double wheelRadius = 3.0, //inches
			ticksPerRevolution = 256 * 4, //encoder ticks, multiplied by 4 because quadrature encoders do 4 pulses per count
			distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
			deadband = 0.08, //-1 to 1
			rampRateAuto = 1, //Seconds to ramp from 0 to full
			rampRateTeleop = 0.25,
			maxTicksPer100ms = 825,//This is the max speed in native units per 100 ms of the motors (percent output 100%)
			maxError = 400,
			minimumSpeed = 75,//ticks per 100 ms
			kP_StraightDriving = 50;
	
	//Turning PID
	public static double kP_Turning = 0.01,
			kD_Turning = 0,
			kI_Turning = 0,
			iZone = 8, //Degree range in which I-gain applies
			turningTolerance = 4, //Degrees
			steadyTurningIterations = 5, //Iterations to exit turning PID loops
			maxTurningOutput = 0.9,
			minimumTurningOutput = 0;
	
	//Driving Speeds in Feet Per Second (FPS)
		public static final double velocityMax = getFPS(maxTicksPer100ms),
				velocityFast = 10,
				velocityMedium = 7,
				velocitySlow = 5.5,
				velocityTurningLeft = 6,
				velocityTurningRight = 6.3,
				velocityVerySlow = 1.5;
	
	//Elevator Constants
	public static final double elevatorEquilibriumSpeedWithCube = 0.31,
			elevatorEquilibriumSpeedNoCube = 0.29,
			elevatorMaxSpeedUp = 1,
			elevatorMaxSpeedDown = -0.25,
			elevatorMinimumSpeedUp = 0.45,
			elevatorMinimumSpeedDown = 0.15,
			ticksPerRotationElevator = 2048,
			pulleyDiameter = 2.75,
			verticalInchesPerTick = 2 * (pulleyDiameter * Math.PI) / ticksPerRotationElevator,
			elevatorTolerance = 2, //Inches
			maximumHeight = 70.8,
			floorHeight = 0.0,
			autoDriveHeight = 5,
			switchHeight = 30,
			scaleHeight = 70.5, 
			elevatorRampTime = 0.2; //seconds
	
	//PID for DriveTrain
	public static double kP_L = (1023 * 0.05)/225,//-(1023 * 0.1)/maxError,
	        kP_R = (1023 * 0.05)/225,
			kD = 0,//kP * 10,//1023.0/maxError,
			kI = 0,//1.023/maxError,
			kF_R = 1023/825,
			kF_L = 1023/825;
	
	public static void getPreferences() {
		Preferences prefs = Preferences.getInstance();
		kP_Turning = prefs.getDouble("kP_Turning", kP_Turning);
		kI_Turning = prefs.getDouble("kI_Turning", kI_Turning);
		kD_Turning = prefs.getDouble("kD_Turning", kD_Turning);
		iZone = prefs.getDouble("iZone", iZone); //Degree range in which I-gain applies
		turningTolerance = prefs.getDouble("turningTolerance", turningTolerance); //Degrees
		steadyTurningIterations = prefs.getDouble("steadyTurningIterations", steadyTurningIterations); //Iterations to exit turning PID loops
		maxTurningOutput = prefs.getDouble("maxTurningOutput", maxTurningOutput);
		minimumTurningOutput = prefs.getDouble("minimumTurningOutput", minimumTurningOutput);
	}
	
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
