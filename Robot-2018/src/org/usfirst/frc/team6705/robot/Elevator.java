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
		elevatorMotor.setSafetyEnabled(false);
		
		//pid.enable();
	}
	
	public static double convertTicksToVerticalInches(double ticks) {
		return ticks * verticalInchesPerTick;
		//Implement appropriate function to convert, taking into account the widening/thinning of axle as pulley rolls
	}
	
	public static double getCurrentPosition() {
		return convertTicksToVerticalInches(encoder.get()) + floorPosition;
	}
	
	public static void liftElevator() {
		elevatorMotor.set(elevatorSpeed);
	}
	
	public static void lowerElevator() {
		elevatorMotor.set(-elevatorSpeed);
	}
	
	public static void moveElevator(double speed) {
		elevatorMotor.set(speed);
	}
	
	public static void stop() {
		elevatorMotor.set(0);
	}
	
	public static void moveElevatorToHeight(double inches) {
		
		double startingHeight = getCurrentPosition();
		double inchesToMove = Math.abs(inches - startingHeight);
		double maxVelocity = elevatorSpeed;
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
				
			moveElevator(direction * scaledFraction * maxVelocity);
		}
		
		stop();
		
	}
	
	public static void toFloor() {
		moveElevatorToHeight(floorPosition);
	}
			
	public static void toSwitch() {
		moveElevatorToHeight(switchPosition);
	}
	
	public static void toScale() {
		moveElevatorToHeight(scalePosition);
	}
	
}
