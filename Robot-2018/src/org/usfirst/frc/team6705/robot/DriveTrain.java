package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
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
		
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		//TODO: Perform any other talon and victor config here
		
		gyro.reset();
		resetEncoders();
	}
	
	//Tank drive for teleop control
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		leftTalon.set(ControlMode.PercentOutput, leftSpeed);
		rightTalon.set(ControlMode.PercentOutput, rightSpeed);
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
			if (currentAngle < Math.abs(degrees) + 2 && currentAngle > Math.abs(degrees) - 2) {
				isDoneTurning = true;
			}
		}
		stop();
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
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
