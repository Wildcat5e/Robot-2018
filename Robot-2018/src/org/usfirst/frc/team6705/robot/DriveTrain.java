package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team6705.robot.Constants.*;

public class DriveTrain {

	static WPI_TalonSRX leftTalon = new WPI_TalonSRX(leftTalonChannel);
	static WPI_TalonSRX rightTalon = new WPI_TalonSRX(rightTalonChannel);
	static WPI_VictorSPX leftVictor = new WPI_VictorSPX(leftVictorChannel);
	static WPI_VictorSPX rightVictor = new WPI_VictorSPX(rightVictorChannel);
	
	static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	private static int turningStableTicks = 0;
	private static double previousTurningError = 0;
	private static double turningSum = 0;
	
	public static void setup() {
		leftVictor.follow(leftTalon);
		rightVictor.follow(rightTalon);
		
		//leftTalon.setSafetyEnabled(true);
		//rightTalon.setSafetyEnabled(true);
		
		leftTalon.setNeutralMode(NeutralMode.Brake);
		rightTalon.setNeutralMode(NeutralMode.Brake);
		leftVictor.setNeutralMode(NeutralMode.Brake);
		rightVictor.setNeutralMode(NeutralMode.Brake);
		
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
		leftVictor.setInverted(true);
		rightVictor.setInverted(false);
		
		leftTalon.configOpenloopRamp(rampRateTeleop, 0);
		rightTalon.configOpenloopRamp(rampRateTeleop, 0);
		leftTalon.configClosedloopRamp(rampRateAuto, 0);
		rightTalon.configClosedloopRamp(rampRateAuto, 0);
		
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(true);
		
		leftTalon.config_kP(0, kP_L, 0);
		leftTalon.config_kI(0, kI, 0);
		leftTalon.config_kD(0, kD, 0);
		leftTalon.config_kF(0, kF_L, 0);
		rightTalon.config_kP(0, kP_R, 0);
		rightTalon.config_kI(0, kI, 0);
		rightTalon.config_kD(0, kD, 0);
		rightTalon.config_kF(0, kF_R, 0);
		
		rightTalon.configMotionProfileTrajectoryPeriod(10, 0);
		leftTalon.configMotionProfileTrajectoryPeriod(10, 0);
		rightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		leftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		
		gyro.reset();
		resetEncoders();
	}
	
	public static void reverseDriveTrain() {
		leftTalon.setInverted(false);
		rightTalon.setInverted(true);
		leftVictor.setInverted(false);
		rightVictor.setInverted(true);
	}
	
	public static void undoReverseDriveTrain() {
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
		leftVictor.setInverted(true);
		rightVictor.setInverted(false);
	}
	
	//Tank drive for teleop control
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		leftSpeed = applyDeadband(leftSpeed);
		rightSpeed = applyDeadband(rightSpeed);
		
		
		double leftTarget = Math.copySign(leftSpeed * leftSpeed * leftSpeed * -1, leftSpeed) * maxTicksPer100ms;
		double rightTarget = Math.copySign(rightSpeed * rightSpeed * rightSpeed * -1, rightSpeed) * maxTicksPer100ms;
		
		//System.out.print("Speed Left:" + getFPS(leftTarget));
		//System.out.println("Speed right:" + getFPS(rightTarget));
		
		setVelocity(leftTarget, rightTarget);
		
		//Percent Output Mode
		//setSpeed(leftSpeed * leftSpeed * leftSpeed, rightSpeed * rightSpeed * rightSpeed);
		
		
	}
	
	public static double applyDeadband(double speed) {
		if ((speed < deadband && speed > 0) || (speed > -deadband && speed < 0)) {
			speed = 0;
		} else if (speed > 0.92) {
			speed = 1;
		} else if (speed < -0.92) {
			speed = -1;
		}
		return speed;
	}

	//Autonomous move method
	public static boolean moveByDistance(double inches, double degrees, double velocity) {
		System.out.print("Move By Distance ");
		double targetEncoderTicks = Math.abs(convertInchesToTicks(inches));
		double ticksSoFar = Math.abs((leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0))/2);
		double maxVelocity = convertVelocity(velocity);
		
		if (ticksSoFar >= targetEncoderTicks) {
			DriveTrain.stop();
			resetEncoders();
			gyro.reset();
			return true;
		}
		
		int direction = (inches > 0) ? 1 : -1;
		
		double ticksRemaining = targetEncoderTicks - ticksSoFar;
		double fractionRemaining = ticksRemaining/targetEncoderTicks;
		double scaledFraction = fractionRemaining * 3; //Start slowing down 2/3 of the way there
		if (scaledFraction > 1) {
			scaledFraction = 1;
		}
		
		double degreeErrorLeft = (getGyro() - degrees > 0) ? getGyro() - degrees : 0;
		double degreeErrorRight = (getGyro() - degrees < 0) ? degrees - getGyro() : 0;

		double velocityLeft = maxVelocity + (kP_Angle * degreeErrorLeft * direction);
		double velocityRight = maxVelocity + (kP_Angle * degreeErrorRight * direction);
		
		double scaledSpeedR = scaledFraction * velocityRight;
		double scaledSpeedL = scaledFraction * velocityLeft;
		
		if (scaledSpeedR < minimumSpeed + (kP_Angle * degreeErrorRight * direction)) {
			scaledSpeedR = minimumSpeed + + (kP_Angle * degreeErrorRight * direction);
		}
		if (scaledSpeedL < minimumSpeed + (kP_Angle * degreeErrorLeft * direction)) {
			scaledSpeedL = minimumSpeed  + (kP_Angle * degreeErrorLeft * direction);
		}
		
		setVelocity(direction * -scaledSpeedL, direction * -scaledSpeedR);
		//setSpeed(direction * -scaledSpeedL, direction * -scaledSpeedR);
		return false;
	}

	
	public static boolean moveByDistance(double inches, double velocity) {
		return moveByDistance(inches, 0, velocity);
	}
	

	//Autonomous turn method
	public static boolean turnDegrees(double degrees) {
		//Positive degrees -> counterclockwise; negative degrees -> clockwise
		System.out.println("Turning with current gyro angle " + getGyro() + " and target " + degrees);
		
		int turnMultiplier = (degrees < 0) ? 1 : -1;

		double currentAngle = getGyro();
		double error = degrees - currentAngle;
		double absoluteError = Math.abs(error);
		if (previousTurningError == 0) {
			previousTurningError = degrees;
		}
		
		if (absoluteError <= turningTolerance) {
			turningStableTicks++;
			if (turningStableTicks >= minimumSteadyTurningIterations) {
				System.out.println("Attempting to stop at gyro angle: " + getGyro());

				DriveTrain.stop();
				gyro.reset();
		
				turningStableTicks = 0;
				previousTurningError = 0;
				turningSum = 0;

				return true;
			}	
		} else {
			turningStableTicks = 0;
		}

		double output = error * kP_Turning + (error - previousTurningError) * kD_Turning;
		if (absoluteError < 5) {
			turningSum += error;
			output += turningSum * kI_Turning;
		} else {
			turningSum = 0;
		}

		previousTurningError = error;

		if (output > maxTurningOutput) {
			output = maxTurningOutput;
		} else if (output < minimumTurningOutput) {
			output = minimumTurningOutput;
		}
		System.out.println("Setting turning speed: " + output);
		setSpeed(-1 * turnMultiplier * output, turnMultiplier * output);
		return false;
	}

	//***************************************//
	
	public static void setupMotionProfile(MotionProfile profile) {
		profile.setup();
		profile.startFilling();
	}
	
	public static void startMotionProfile(MotionProfile profile) {
		profile.periodic();
		profile.startMotionProfile();
	}

	public static boolean runMotionProfile(MotionProfile profile) {
		SetValueMotionProfile setValue = profile.getSetValue();
		leftTalon.set(ControlMode.MotionProfile, setValue.value);
		rightTalon.set(ControlMode.MotionProfile, setValue.value);
		profile.periodic();
		
		if (profile.isMotionProfileComplete()) {
			return true;
		}
		return false;
	}
	
	//***************************************//
	
	public static void setVelocity(double left, double right) {
	    double elevatorHeight = Elevator.encoder.get();
	    double scale = 1; 
	    if (elevatorHeight > 3000) {
	        scale = 1 - (elevatorHeight/(maxElevatorTicks * 3.25));
	    }
		
		leftTalon.set(ControlMode.Velocity, left * scale);
		rightTalon.set(ControlMode.Velocity, right * scale);
		
		System.out.println("Setting velocities L: " + left + " R: " + right);
		System.out.println("Actual speed L: " + leftTalon.getSelectedSensorVelocity(0) + " R: " + rightTalon.getSelectedSensorVelocity(0));
	}
	
	public static void setSpeed(double left, double right) {
		leftTalon.set(ControlMode.PercentOutput, left);
		rightTalon.set(ControlMode.PercentOutput, right);
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
		leftVictor.set(ControlMode.PercentOutput, 0);
		rightVictor.set(ControlMode.PercentOutput, 0);
	}
	
	
	public static boolean wait(double time, double previousTime) {
		if (Robot.timer.get() - previousTime >= time) {
			return true;
		}
		return false;
	}
	
	public static double getGyro() {
		return -gyro.getAngle();
	}
	
	public static void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0, 0, 0);
		rightTalon.setSelectedSensorPosition(0, 0, 0);
	}
	
}
