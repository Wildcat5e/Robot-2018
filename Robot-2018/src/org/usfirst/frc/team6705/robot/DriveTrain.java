package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

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
		
		leftTalon.setSafetyEnabled(true);
		rightTalon.setSafetyEnabled(true);
		
		leftTalon.setNeutralMode(NeutralMode.Brake);
		rightTalon.setNeutralMode(NeutralMode.Brake);
		leftVictor.setNeutralMode(NeutralMode.Brake);
		rightVictor.setNeutralMode(NeutralMode.Brake);
		
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
		
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
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

	public static void moveByDistance(double inches) {
		//Reset encoders
		resetEncoders();
		
		//Convert argument from inches to encoder ticks
		double targetEncoderTicks = convertInchesToTicks(inches);
		
		int direction = 1;
		if (inches < 0) {
			direction = -1;
		}
		
		double maxVelocity = convertFPSToTicksPer100MS(velocityMedium);
		double ticksRemaining;
		
		while (direction * leftTalon.getSelectedSensorPosition(0) < direction * targetEncoderTicks && DriverStation.getInstance().isAutonomous()) {
			ticksRemaining = targetEncoderTicks - leftTalon.getSelectedSensorPosition(0);
			double fractionRemaining = ticksRemaining/targetEncoderTicks;
			double scaledFraction = fractionRemaining * 3; //Start slowing down 2/3 of the way there
			if (scaledFraction > 1) {
				scaledFraction = 1;
			} else if (scaledFraction < 0.05) {
				scaledFraction = 0.05;
			}
			
			double velocity = scaledFraction * maxVelocity;
			setVelocity(direction * velocity, direction * velocity);
		} 
		
		stop(); //Stop driving when loop finishes
	}
		
	
	public static void setVelocity(double left, double right) {
		leftTalon.set(ControlMode.Velocity, left);
		rightTalon.set(ControlMode.Velocity, right);
	}
	
	public static void turnDegrees(double degrees) {
		//Positive degrees -> counterclockwise; negative degrees -> clockwise
		
		double maxVelocity = convertFPSToTicksPer100MS(velocityTurning);
		int turnMultiplier = (degrees < 0) ? -1 : 1;
		boolean isDoneTurning = false;
		double currentAngle;
		gyro.reset();
		
		while (!isDoneTurning && DriverStation.getInstance().isAutonomous()) {
			setVelocity(-1 * turnMultiplier * maxVelocity, turnMultiplier * maxVelocity);
			currentAngle = Math.abs(gyro.getAngle());
			if (currentAngle < Math.abs(degrees) + turningTolerance && currentAngle > Math.abs(degrees) - turningTolerance) {
				isDoneTurning = true;
			}
		}
		stop();
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
	}
	
	public static void wait(double time) {
		Timer timer = new Timer();
		timer.start();
		while (timer.get() < time) {
			stop();
		}
	}
	
	public static void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0, 0, 0);
		rightTalon.setSelectedSensorPosition(0, 0, 0);
	}
	
	public static void moveTillStall() {
		while(leftTalon.getOutputCurrent() < 20) {	
			setVelocity(velocityMedium, velocityMedium);
		}
	}
	
}
