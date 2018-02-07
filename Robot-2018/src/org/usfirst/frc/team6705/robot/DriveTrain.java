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
		
		
		//TODO: Perform any other talon and victor config here
		
		gyro.reset();
		resetEncoders();
	}
	
	//Tank drive for teleop control
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		leftSpeed = applyDeadband(leftSpeed);
		rightSpeed = applyDeadband(rightSpeed);
		
		leftTalon.set(ControlMode.PercentOutput, Math.copySign(leftSpeed * leftSpeed, leftSpeed));
		rightTalon.set(ControlMode.PercentOutput, Math.copySign(rightSpeed * rightSpeed, rightSpeed));
	}
	
	public static double applyDeadband(double speed) {
		if ((speed < deadband && speed > 0) || (speed > -deadband && speed < 0)) {
			speed = 0;
		}
		return speed;
	}

	//Autonomous move method
	public static boolean moveByDistance(double inches, double velocity) {
		SmartDashboard.putNumber("Auto State", 1.2);
		System.out.print("Move By Distance");
		double targetEncoderTicks = convertInchesToTicks(inches);
		double ticksSoFar = leftTalon.getSelectedSensorPosition(0);
		double maxVelocity = convertFPSToTicksPer100MS(velocity);
		
		if (ticksSoFar >= targetEncoderTicks) {
			resetEncoders();
			return true;
		}
		
		int direction = 1;
		if (inches < 0) {
			direction = -1;
		}
		
		double ticksRemaining = convertInchesToTicks(targetEncoderTicks - ticksSoFar);
		double fractionRemaining = ticksRemaining/targetEncoderTicks;
		double scaledFraction = fractionRemaining * 3; //Start slowing down 2/3 of the way there
		if (scaledFraction > 1) {
			scaledFraction = 1;
		} else if (scaledFraction < 0.05) {
			scaledFraction = 0.05;
		}
			
		double speed = scaledFraction * maxVelocity;
		setVelocity(direction * speed, direction * speed);
		return false;
	}
	
	//Move until runs into switch
	public static boolean moveTillStall() {
		if (leftTalon.getOutputCurrent() > stallCurrent) {
			return true;
		}
		
		double velocity = convertFPSToTicksPer100MS(velocityMedium);
		setVelocity(velocity, velocity);
		return false;
	}
	
	//Autonomous turn method
	public static boolean turnDegrees(double degrees) {
		//Positive degrees -> counterclockwise; negative degrees -> clockwise
		double maxVelocity = convertFPSToTicksPer100MS(velocityTurning);
		int turnMultiplier = (degrees < 0) ? -1 : 1;
		double currentAngle = gyro.getAngle();
		if (currentAngle < degrees + turningTolerance && currentAngle > degrees - turningTolerance) {
			gyro.reset();
			return true;
		}
		
		setVelocity(-1 * turnMultiplier * maxVelocity, turnMultiplier * maxVelocity);
		return false;
	}
	
	
	public static void setVelocity(double left, double right) {
		leftTalon.set(ControlMode.Velocity, left);
		rightTalon.set(ControlMode.Velocity, right);
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
	
	public static void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0, 0, 0);
		rightTalon.setSelectedSensorPosition(0, 0, 0);
	}
	
}
