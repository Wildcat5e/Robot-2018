/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.Preferences;

public class Constants {
	
	//Motor Controllers
	public static final int leftTalonChannel = 1,
			rightTalonChannel = 0,
			leftVictorChannel = 0,
			rightVictorChannel = 1,
			leftIntakeSparkChannel = 2,
			rightIntakeSparkChannel = 4,
			elevatorSpark1 = 0,
			elevatorSpark2 = 1;
	
	//Solenoids
	public static final int intakeSolenoidA = 0,
			intakeSolenoidB = 1,
			leftRampSolenoidA = 0,
			leftRampSolenoidB = 1,
			rightRampSolenoidA = 2,
			rightRampSolenoidB = 3;
	
	//Joysticks
	public static final int driveStickChannel = 0,
			dPadChannel = 0;
	
	
	//Auto Starting Positions
	public static final String left = "left",
			middle = "middle",
			right = "right";
	
	//Sensors
	public static final int elevatorEncoderSourceA = 0,
			elevatorEncoderSourceB = 1,
			elevatorLimitSwitchDIO = 2;
	
	//Rollers
	public static final double timeToRollIn = 1.5, //seconds
		timeToRollOut = 1.5, //seconds
		rollersSpeed = 0.5;//[-1, 1]
		
	
	//Drive Train
	public static final double wheelRadius = 3.0, //inches
			ticksPerRevolution = 256 * 4, //encoder ticks, multiplied by 4 because quadrature encoders do 4 pulses per count
			distancePerTick = (wheelRadius * 2.0 * Math.PI)/ticksPerRevolution, //inches per tick
			turningTolerance = 2.5, //Degrees
			deadband = 0.08, //-1 to 1
			rampRateAuto = 1, //Seconds to ramp from 0 to full
			rampRateTeleop = 0.25,
			stallCurrent = 60, // TODO  This number needs to be defined in testing (estimate);
			maxTicksPer100ms = 850,//This is the max speed in native units per 100 ms of the motors (percent output 100%)
			maxError = 400,
			minimumSpeed = 900,//ticks per 100 ms
			minimumTurningSpeed = 950,
			kP_Angle = 70;
	
	//Driving Speeds in Feet Per Second (FPS)
		public static final double velocityMax = getFPS(maxTicksPer100ms),
				velocityFast = 9,
				velocityMedium = 6,
				velocitySlow = 3,
				velocityTurning = 3;
	
	//Elevator Constants
	public static final double elevatorConstantSpeedCube = 0.37,
			elevatorConstantSpeedNoCube = 0.35,
			elevatorMaxSpeedUp = 1,
			elevatorMaxSpeedDown = 0.01,
			elevatorMinimumSpeedUp = 0.4,
			elevatorMinimumSpeedDown = 0.15,
			ticksPerRotationElevator = 2048,
			verticalInchesPerTick = (3 * Math.PI) / ticksPerRotationElevator,
			elevatorTolerance = 1, //Inches
			maximumHeight = 90,
			floorHeight = 7.0,
			switchHeight = 25.0,
			scaleHeight = 82.0,
			elevatorRampTime = 0.2, //seconds
			elevatorDecelerationDistance = 8,
			ropeThickness = 0; //inches
	
	//PID for DriveTrain
	public static double kP_L = (1023 * 0.05)/240,//-(1023 * 0.1)/maxError,
	        kP_R = (1023 * 0.05)/200,
			kD = kP_L = 0,//kP * 10,//1023.0/maxError,
			kI = 0,//1.023/maxError,
			kF_R = 1023/854,
			kF_L = 1023/764;
	
	//PID for Elevator
	public static double kP_Lift = 0.1,
		kD_Lift = 0.01,
		kI_Lift = 0.01,
		kF_Lift = 0.1;
	
	public static void setup() {
		/*Preferences prefs = Preferences.getInstance();
		kP = prefs.getDouble("kP_R", 0.0001);
		kD = prefs.getDouble("kD", 0);
		kI = prefs.getDouble("kI", 0);
		kF_L = prefs.getDouble("kF_L", 0.1);
		kF_R = prefs.getDouble("kF_R", 0.1);
		kP_Lift = prefs.getDouble("kP_Lift", 0);
		kI_Lift = prefs.getDouble("kI_Lift", 0);
		kD_Lift= prefs.getDouble("kD_LIft", 0);
		kF_Lift = prefs.getDouble("kF_LIft", 0);*/
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
