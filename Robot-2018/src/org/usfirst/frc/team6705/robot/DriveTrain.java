package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team6705.robot.Constants.*;

public class DriveTrain {

	static WPI_TalonSRX leftTalon = new WPI_TalonSRX(leftTalonChannel);
	static WPI_TalonSRX rightTalon = new WPI_TalonSRX(rightTalonChannel);
	static WPI_VictorSPX leftVictor = new WPI_VictorSPX(leftVictorChannel);
	static WPI_VictorSPX rightVictor = new WPI_VictorSPX(rightVictorChannel);
	
	static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
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
		
		leftTalon.configOpenloopRamp(rampRate, 0);
		rightTalon.configOpenloopRamp(rampRate, 0);
		leftTalon.configClosedloopRamp(rampRate, 0);
		rightTalon.configClosedloopRamp(rampRate, 0);
		
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(true);
		
		leftTalon.config_kP(0, kP_L, 0);
		leftTalon.config_kI(0, kI, 0);
		leftTalon.config_kD(0, kD, 0);
		leftTalon.config_kF(0, kF, 0);
		rightTalon.config_kP(0, kP_R, 0);
		rightTalon.config_kI(0, kI, 0);
		rightTalon.config_kD(0, kD, 0);
		rightTalon.config_kF(0, kF, 0);
		//TODO: Perform any other talon and victor config here
		
		gyro.reset();
		resetEncoders();
	}
	
	//Tank drive for teleop control
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		leftSpeed = applyDeadband(leftSpeed);
		rightSpeed = applyDeadband(rightSpeed);
		
		System.out.println("Left Speed: " + leftSpeed + " Right Speed: " + rightSpeed);

		double leftTarget = Math.copySign(leftSpeed * leftSpeed, leftSpeed) * maxTicksPer100ms;
		double rightTarget = Math.copySign(rightSpeed * rightSpeed, rightSpeed) * maxTicksPer100ms;
		
		setVelocity(leftTarget, rightTarget);
		
		/* Percent Output Mode
		leftTalon.set(ControlMode.PercentOutput, Math.copySign(leftSpeed * leftSpeed, leftSpeed));
		rightTalon.set(ControlMode.PercentOutput, Math.copySign(rightSpeed * rightSpeed, rightSpeed));
		*/
		
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
	public static boolean moveByDistance(double inches, double velocity) {
		System.out.print("Move By Distance ");
		double targetEncoderTicks = Math.abs(convertInchesToTicks(inches));
		double ticksSoFar = Math.abs(leftTalon.getSelectedSensorPosition(0));
		double maxVelocity = convertVelocity(velocity);
		
		if (ticksSoFar >= targetEncoderTicks) {
			resetEncoders();
			return true;
		}
		
		int direction = (inches > 0) ? -1 : 1;
		
		double ticksRemaining = targetEncoderTicks - ticksSoFar;
		double fractionRemaining = ticksRemaining/targetEncoderTicks;
		double scaledFraction = fractionRemaining * 3; //Start slowing down 2/3 of the way there
		if (scaledFraction > 1) {
			scaledFraction = 1;
		} /*else if (scaledFraction < 0.4) {
			scaledFraction = 0.4;
		}*/
		System.out.println("Scaled Fraction " + scaledFraction);	
		double scaledSpeed = scaledFraction * maxVelocity;
		if (scaledSpeed < minimumSpeed) {
			scaledSpeed = minimumSpeed;
		}
		setVelocity(direction * scaledSpeed, direction * scaledSpeed);
		return false;
	}
	
	//Move until runs into switch
	public static boolean moveTillStall() {
		if (leftTalon.getOutputCurrent() > stallCurrent) {
			return true;
		}
		
		double velocity = convertVelocity(velocityMedium);
		setVelocity(velocity, velocity);
		return false;
	}
	
	//Autonomous turn method
	public static boolean turnDegrees(double degrees) {
		//Positive degrees -> counterclockwise; negative degrees -> clockwise
		System.out.println("Starting gyro angle: " + gyro.getAngle());
		double maxVelocity = convertVelocity(velocityTurning);
		int turnMultiplier = (degrees < 0) ? 1 : -1;
		double currentAngle = getGyro();
		
		if (currentAngle < degrees + turningTolerance && currentAngle > degrees - turningTolerance) {
			System.out.println("Attempting to stop at gyro angle: " + getGyro());
			gyro.reset();
			return true;
		}
		
		double degreesRemaining = Math.abs(degrees) - Math.abs(currentAngle);
		double fractionRemaining = Math.abs(degreesRemaining/degrees);
		double scaledFraction = fractionRemaining * 1.75; //Uncomment the * 2 to decelerate halfway through the turn
		if (scaledFraction > 1) {
			scaledFraction = 1;
		} else if (scaledFraction < 0.35) {
			scaledFraction = 0.35;
		}
		System.out.println("Scaled Fraction: " + scaledFraction);
		double scaledSpeed = maxVelocity * scaledFraction;
		setVelocity(-1 * turnMultiplier * scaledSpeed, turnMultiplier * scaledSpeed);
		return false;
	}
	
	
	public static void setVelocity(double left, double right) {
		//leftTalon.config_kF(0,  1023  * left/maxTicksPer100ms, 0);
		//rightTalon.config_kF(0, 1023 * right/maxTicksPer100ms, 0);
		
		
		leftTalon.set(ControlMode.Velocity, left);
		rightTalon.set(ControlMode.Velocity, right);
		
		System.out.println("Setting velocities L: " + left + " R: " + right);
		System.out.println("Actual speed L: " + leftTalon.getSelectedSensorVelocity(0) + " R: " + rightTalon.getSelectedSensorVelocity(0));
		System.out.println("Error L: " + (Math.abs(leftTalon.getSelectedSensorVelocity(0)) - Math.abs(left)) + " R: " + (Math.abs(rightTalon.getSelectedSensorVelocity(0)) - Math.abs(right)));
		//System.out.println("kF Left: " + (1023  * left/maxTicksPer100ms));
		//System.out.println("kF Right: " + (1023  * right/maxTicksPer100ms));
		//System.out.println("L-R Difference: " + (leftTalon.getSelectedSensorVelocity(0) - rightTalon.getSelectedSensorVelocity(0)));
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
	}
	
	/*
	public static boolean wait(double time) {
		if (Robot.timer.get() >= time) {
			return true;
		}
		return false;
	}*/
	
	public static double getGyro() {
		return -gyro.getAngle();
	}
	
	public static void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0, 0, 0);
		rightTalon.setSelectedSensorPosition(0, 0, 0);
	}
	
}
