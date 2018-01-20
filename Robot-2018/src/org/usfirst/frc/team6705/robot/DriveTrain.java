package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
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
		
		gyro.reset();

		
	}
	
	
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		//TODO: look at squred inputs
		leftTalon.set(ControlMode.PercentOutput, leftSpeed);
		rightTalon.set(ControlMode.PercentOutput, rightSpeed);
	}

	public static void moveByDistance(double inches) {
		double targetEncoderTicks = (inches/(2 * Math.PI * wheelRadius)) * pulsesPerRotation;
		double maxVelocity = (autoForwardSpeedRPM * pulsesPerRotation) / 600.0;
		double ticksRemaining;
		
		do {
			ticksRemaining = targetEncoderTicks - leftTalon.getSelectedSensorPosition(0);
			double velocity = (ticksRemaining/targetEncoderTicks) * maxVelocity;
			setVelocity(velocity, velocity);
		} while (leftTalon.getSelectedSensorPosition(0) < targetEncoderTicks);
		
		stop();

	}
		
	
	public static void setVelocity(double left, double right) {
		leftTalon.set(ControlMode.Velocity, left);
		rightTalon.set(ControlMode.Velocity, right);
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
	}
	
	public static void turnDegrees(double degrees) {
		
	}
	
}
