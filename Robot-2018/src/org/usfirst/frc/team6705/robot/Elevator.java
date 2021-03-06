package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;
//import org.usfirst.frc.team6705.robot.PIDMotor;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Elevator /*extends PIDSubsystem*/ {
	
	static Spark spark1 = new Spark(elevatorSpark1);
	static Spark spark2 = new Spark(elevatorSpark2);
	
	static SpeedControllerGroup motor = new SpeedControllerGroup(spark1, spark2);

	static Encoder encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
	
	static DigitalInput limitSwitch = new DigitalInput(elevatorLimitSwitchDIO);
	
	public static boolean hasCompletedLift = false;
	
	//static PIDController pid = new PIDController(kP_Lift, kI_Lift, kD_Lift, kF_Lift, encoder, motor);
	
	//static PIDMotor motor1;
	//static PIDMotor motor2;
	
	public static void setup() {
	    //encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
		encoder.reset();
		encoder.setDistancePerPulse(verticalInchesPerTick);
		
	}
	
	public static double convertTicksToVerticalInches(double ticks) {
		return ticks * verticalInchesPerTick;
	}
	
	public static double getCurrentPosition() {
		return convertTicksToVerticalInches(encoder.get()) + floorHeight;
	}
	
	public static void set(double speed) { //Takes in a value from -1 to 1, scales it to the values that work for the elevator, and sets the motor to that speed
	    if ((speed >= 0 && getCurrentPosition() < maximumHeight) || (speed < 0 && !isAtFloor())) {
            System.out.println("Setting lift speed (unscaled): " + speed);
            if (speed < 0) {
                double downSpeed = (speed * (elevatorMinimumSpeedDown - elevatorMaxSpeedDown)) + elevatorMinimumSpeedDown;
                System.out.println("Setting elevator speed down: " + downSpeed);
                motor.set(downSpeed);
            } else if (speed >= 0) {
                double upSpeed = (speed * (1 - elevatorMinimumSpeedUp)) + elevatorMinimumSpeedUp;
                System.out.println("Setting elevator speed up: " + upSpeed);
                motor.set(upSpeed);
            }
        } else {
            System.out.println("REACHED A LIMIT");
            //stop();
        }
	}
	
	public static void setTeleop(double speed, int intervalsCounted) {
		double intervals = (intervalsCounted > elevatorRampTime * 50) ? elevatorRampTime * 50 : intervalsCounted;
		/*if (speed < 0 && getCurrentPosition() < floorHeight + elevatorDecelerationDistance) {
			Elevator.set(speed * ((getCurrentPosition() - floorHeight)/elevatorDecelerationDistance));
		} else if (speed > 0 && getCurrentPosition() > maximumHeight - elevatorDecelerationDistance) {
			Elevator.set(speed * ((maximumHeight - getCurrentPosition())/elevatorDecelerationDistance));
		} */ if (intervals < elevatorRampTime * 50 && speed > 0) {
		    double actualSpeed = (intervals/(elevatorRampTime * 50)) * speed * elevatorMaxSpeedUp;
		    System.out.println("Setting teleop speed " + actualSpeed);
			Elevator.set(actualSpeed);
		} else {
			set(speed);
		}
	}
	
	public static void maintainHeight(double height) {
	    if (getCurrentPosition() < height + elevatorTolerance) {
	        System.out.println("Trying to maintain height " + height + "Current Position is " + getCurrentPosition());
	        double speed = (Intake.solenoid.get() == DoubleSolenoid.Value.kReverse) ? elevatorEquilibriumSpeedWithCube : elevatorEquilibriumSpeedNoCube;//elevatorMinimumSpeedUp * (height - getCurrentPosition());
	        //speed = (height - getCurrentPosition()) * 0.1;
	        //System.out.println("Equilibrium speed: " + speed);
	        motor.set(speed);
	    } /*else {
	        motor.set(0);
	    }*/
	}
	
	public static void stop() {
		spark1.set(0);
		spark2.set(0);
	}
	
	public static boolean isAtFloor() {
		if (limitSwitch.get() == false) {
			return true;
		} else {
			return false;
		}
	}
	
	public static void moveToHeight(double targetHeight, double distanceToLift, int direction) {
		//int direction = (currentHeight > targetHeight) ? -1 : 1;

		double distanceRemaining = Math.abs(getCurrentPosition() - targetHeight);
		double fractionRemaining = Math.abs(distanceRemaining/distanceToLift);
		System.out.println("Fraction Remaining: " + fractionRemaining);
		System.out.println("Distance Remaining: " + distanceRemaining);
		
		if (fractionRemaining > 1) {
			fractionRemaining = 1;
		} 
		
		double scaledFraction = fractionRemaining * 1.25;
		
		double fractionLifted = 1 - fractionRemaining;
		if (fractionLifted < 0.01) {
			fractionLifted = 0.01;
		}

		double scaledFractionLifted = fractionLifted * 5;
		
		if (fractionLifted < 0.2) {
			scaledFraction = scaledFractionLifted;
		} else if (scaledFraction > 1) {
			scaledFraction = 1;
		}
		
		System.out.println("Moving to height " + targetHeight + " with current height " + Elevator.getCurrentPosition());
		Elevator.set(direction * scaledFraction);
	}
	
	public static boolean moveToHeightAuto(double targetHeight, double totalDistanceToLift, int direction) {
		double currentHeight = getCurrentPosition();
		//int direction = (currentHeight > targetHeight) ? -1 : 1;
		
		double distanceRemaining = currentHeight - targetHeight;
		double absDistance = Math.abs(distanceRemaining);
		System.out.println("Move to height auto");
		
		if ((currentHeight < targetHeight + elevatorTolerance && currentHeight > targetHeight - elevatorTolerance) || hasCompletedLift) {
			hasCompletedLift = true;
			Robot.auto.isLifting = false;
			Robot.auto.previousElevatorHeight = getCurrentPosition();
			System.out.println("AUTO ELEVATOR MOVE DONE");
			return true;
		} else {
			Robot.auto.isLifting = true;
		}
		
		double fractionRemaining = Math.abs(absDistance/totalDistanceToLift);
		if (fractionRemaining > 1) {
			fractionRemaining = 1;
		} 
		
		double scaledFraction = fractionRemaining * 1.25;
		
		double fractionLifted = 1 - fractionRemaining;
		if (fractionLifted < 0.01) {
			fractionLifted = 0.01;
		}
		
        double scaledFractionLifted = fractionLifted * 5;
		
        if (fractionLifted < 0.2) {
			scaledFraction = scaledFractionLifted;
		} else if (scaledFraction > 1) {
			scaledFraction = 1;
		}
        
        Elevator.set(direction * scaledFraction);
        
		return false;
	}
	
	public static boolean moveToFloorAuto(double totalDistanceToLift) {
		double currentHeight = getCurrentPosition();
		int direction = -1;
		
		System.out.println("Move to floor auto");
		
		double distanceRemaining = Math.abs(currentHeight - floorHeight);
		if (Elevator.isAtFloor()) {
			encoder.reset();
			//Elevator.stop();
			Robot.auto.isLifting = false;
			return true;
		} else {
			Robot.auto.isLifting = true;
		}
		
		double fractionRemaining = Math.abs(distanceRemaining/totalDistanceToLift);
		if (fractionRemaining > 1) {
			fractionRemaining = 1;
		}
		
		double scaledFraction = fractionRemaining * 1.25;
		
		double fractionLifted = 1 - fractionRemaining;
		if (fractionLifted < 0.01) {
			fractionLifted = 0.01;
		}
		
        double scaledFractionLifted = fractionLifted * 5;
		
        if (fractionLifted < 0.2) {
        	scaledFraction = scaledFractionLifted;
        } else if (scaledFraction > 1) {
        	scaledFraction = 1;
        }
		
		Elevator.set(direction * scaledFraction);
		return false;
		
	}
	
	public static boolean moveToHeightAfterDriving(double targetHeight, double totalDistanceToLift, int direction, double distanceInches) {
		double leftDistance = DriveTrain.leftTalon.getSelectedSensorPosition(0);
		double rightDistance = DriveTrain.rightTalon.getSelectedSensorPosition(0);
		double averageDistance = (leftDistance + rightDistance)/2;
		
		if (averageDistance < convertInchesToTicks(distanceInches)) {
			System.out.println("Has only driven " + averageDistance + " ticks, so not yet lifting elevator");
			Robot.auto.isLifting = false;
			return false;
		} else {
			System.out.println("Has driven at least " + convertInchesToTicks(distanceInches) + " ticks, so now lifting elevator"); 
			return moveToHeightAuto(targetHeight, totalDistanceToLift, direction);
		}
	}
	
	
	public static enum ElevatorState {
		MANUAL, FLOOR, SWITCH, SCALE, TEST;
	}
	
	
}