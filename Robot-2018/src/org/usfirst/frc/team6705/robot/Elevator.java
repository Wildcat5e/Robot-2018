package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Elevator {
	
	static Spark motor1 = new Spark(elevatorSpark1);
	static Spark motor2 = new Spark(elevatorSpark2);

	static Encoder encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
	
	//static PIDController pid = new PIDController(kP, kI, kD, kF, encoder, elevatorMotor);

	public static void setup() {
		encoder.reset();
		encoder.setDistancePerPulse(verticalInchesPerTick);
		//elevatorMotor.setSafetyEnabled(true);
		
		//pid.enable();
	}
	
	public static double convertTicksToVerticalInches(double ticks) {
		return ticks * verticalInchesPerTick;
		//Implement appropriate function to convert, taking into account the widening/thinning of axle as pulley rolls
	}
	
	public static double getCurrentPosition() {
		return convertTicksToVerticalInches(encoder.get()) + floorHeight;
	}
	
	public static void set(double speed) {
		if ((speed > 0 && getCurrentPosition() < maximumHeight) || (speed < 0 && getCurrentPosition() > floorHeight)) {
			double maxSpeed = (speed < 0) ? elevatorMaxSpeedDown : elevatorMaxSpeedUp;
			motor1.set(speed * maxSpeed);
			motor2.set(speed * maxSpeed);
		} else {
			stop();
		}
	}
	
	public static void stop() {
		motor1.set(0);
		motor2.set(0);
	}
	
	public static void maintainHeight(double height) {
		if (getCurrentPosition() < height) {
			set(0.1 * (height - getCurrentPosition()));
		}
	}
	
	public static void moveToHeight(double targetHeight, double currentHeight, double distanceToLift) {
		int direction = (currentHeight > targetHeight) ? -1 : 1;

		double distanceRemaining = Math.abs(currentHeight - targetHeight);
		double fractionRemaining = distanceRemaining/distanceToLift;
		double scaledFraction = fractionRemaining * 3;
		if (scaledFraction > 1) {
			scaledFraction = 1;
		} else if (scaledFraction < elevatorMinimumSpeedUp && direction == 1) {
			scaledFraction = elevatorMinimumSpeedUp;
		} else if (scaledFraction < elevatorMinimumSpeedDown && direction == -1) {
			scaledFraction = elevatorMinimumSpeedDown;
		}
		
		Elevator.set(direction * scaledFraction);
	}
	
	public static boolean moveToHeightAuto(double targetHeight, double totalDistanceToLift) {
		double currentHeight = getCurrentPosition();
		int direction = (currentHeight > targetHeight) ? -1 : 1;
		
		double distanceRemaining = Math.abs(currentHeight - targetHeight);
		if (distanceRemaining <= 0) {
			Elevator.stop();
			return true;
		}
		
		double fractionRemaining = Math.abs(distanceRemaining/totalDistanceToLift);
		double scaledFraction = fractionRemaining * 3;
		if (scaledFraction > 1) {
			scaledFraction = 1;
		} else if (scaledFraction < elevatorMinimumSpeedUp && direction == 1) {
			scaledFraction = elevatorMinimumSpeedUp;
		} else if (scaledFraction < elevatorMinimumSpeedDown && direction == -1) {
			scaledFraction = elevatorMinimumSpeedDown;
		}
		
		Elevator.set(direction * scaledFraction);
		return false;
	}
	
	public static enum ElevatorState {
		MANUAL, FLOOR, SWITCH, SCALE;
	}
	
}

