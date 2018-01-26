package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Elevator {
	
	static Spark elevatorMotor = new Spark(elevatorSparkChannel);
	static Encoder encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);

	public void liftElevator() {
		elevatorMotor.set(elevatorSpeed);
	}
	
	public void lowerElevator() {
		elevatorMotor.set(-elevatorSpeed);
	}
	
	public void liftElevatorDistance(double inches) {
		
	}
	
}
