package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Elevator {
	
	static Spark elevatorMotor = new Spark(elevatorSparkChannel);
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
			elevatorMotor.set(speed * maxSpeed);
		} else {
			stop();
		}
	}
	
	public static void stop() {
		elevatorMotor.set(0);
	}
	
	public static void maintainHeight(double height) {
		if (getCurrentPosition() < height) {
			set(0.1 * (height - getCurrentPosition()));
		}
	}
	
	public static void moveToHeight(double inches) {
		
		double startingHeight = getCurrentPosition();
		double inchesToMove = Math.abs(inches - startingHeight);
		double inchesRemaining;
		
		int direction = 1;
		if (inches < startingHeight) {
			direction = -1;
		}

		while (getCurrentPosition() * direction < inches * direction) {
			inchesRemaining = inchesToMove - (direction * (getCurrentPosition() - startingHeight));
			double fractionRemaining = inchesRemaining/inchesToMove;
			double scaledFraction = fractionRemaining * 3;
			if (scaledFraction > 1) {
				scaledFraction = 1;
			} else if (scaledFraction < 0.05) {
				scaledFraction = 0.05;
			}
				
			set(direction * scaledFraction);
		}
		
		stop();
		
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
	
	public static void toFloor() {
		moveToHeight(floorHeight);
	}
			
	public static void toSwitch() {
		moveToHeight(switchHeight);
	}
	
	public static void toScale() {
		moveToHeight(scaleHeight);
	}
	
	public static enum ElevatorState {
		MANUAL, FLOOR, SWITCH, SCALE;
	}
	
}

