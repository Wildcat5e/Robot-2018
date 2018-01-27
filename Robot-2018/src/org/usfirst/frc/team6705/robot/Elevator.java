package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Elevator {
	
	static Spark elevatorMotor = new Spark(elevatorSparkChannel);
	static Encoder encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
	
	//static PIDController pid = new PIDController(kP, kI, kD, kF, encoder, elevatorMotor);

	public static void setup() {
		encoder.reset();
		encoder.setDistancePerPulse(verticalInchesPerTick);
		
		//pid.enable();
	}
	
	public static double getCurrentPosition() {
		return encoder.getDistance() + floorPosition;
	}
	
	public static void liftElevator() {
		elevatorMotor.set(elevatorSpeed);
	}
	
	public static void lowerElevator() {
		elevatorMotor.set(-elevatorSpeed);
	}
	
	public static void stop() {
		elevatorMotor.set(0);
	}
	
	public static void moveElevatorToHeight(double inches) {
		
		double startingHeight = getCurrentPosition();
		
		if (inches >= startingHeight) {
			while (getCurrentPosition() < inches) {
				liftElevator();
			}
		} else {
			while (getCurrentPosition() > inches) {
				lowerElevator();
			}
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
