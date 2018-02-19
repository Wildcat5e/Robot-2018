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
	
	//static PIDController pid = new PIDController(kP_Lift, kI_Lift, kD_Lift, kF_Lift, encoder, motor);
	
	//static PIDMotor motor1;
	//static PIDMotor motor2;
	
	public static void setup() {
	    //encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
		encoder.reset();
		encoder.setDistancePerPulse(verticalInchesPerTick);
		
	}
	
	public static double convertTicksToVerticalInches(double ticks) {
		double rotations = Math.floor(encoder.get()/ticksPerRotationElevator);
		double inchesPerTick = verticalInchesPerTick + ((rotations * Math.PI * 2 * ropeThickness)/ticksPerRotationElevator);
		return ticks * inchesPerTick;
	}
	
	public static double getCurrentPosition() {
		return convertTicksToVerticalInches(encoder.get()) + floorHeight;
	}
	
	public static void set(double speed) { //Takes in a value from -1 to 1, scales it to the values that work for the elevator, and sets the motor to that speed
	    if ((speed > 0 && getCurrentPosition() < maximumHeight) || (speed < 0 && getCurrentPosition() > floorHeight)) {
            System.out.println("Setting lift speed (unscaled): " + speed);
            if (speed < 0) {
                double downSpeed = (speed * (elevatorMinimumSpeedDown - elevatorMaxSpeedDown)) + elevatorMinimumSpeedDown;
                System.out.println("Setting elevator speed down: " + downSpeed);
                motor.set(downSpeed);
            } else {
                double upSpeed = (speed * (1 - elevatorMinimumSpeedUp)) * + elevatorMinimumSpeedUp;
                System.out.println("Setting elevator speed up: " + upSpeed);
                motor.set(upSpeed);
            }
        } else {
            stop();
        }
	}
	
	public static void setTeleop(double speed, int intervalsCounted) {
		int intervals = (intervalsCounted > elevatorRampTime * 50) ? 20 : intervalsCounted;
		if (speed < 0 && getCurrentPosition() < floorHeight + elevatorDecelerationDistance) {
			Elevator.set(speed * ((getCurrentPosition() - floorHeight)/elevatorDecelerationDistance));
		} else if (speed > 0 && getCurrentPosition() > maximumHeight - elevatorDecelerationDistance) {
			Elevator.set(speed * ((maximumHeight - getCurrentPosition())/elevatorDecelerationDistance));
		} else if (intervals < 20) {
			Elevator.set((intervals/20) * speed);
		} else {
			set(speed);
		}
	}
	
	public static void maintainHeight(double height) {
	    if (getCurrentPosition() < height + elevatorTolerance) {
	        System.out.println("Trying to maintain height " + height + "Current Position is " + getCurrentPosition());
	        double speed = (Intake.solenoid.get() == DoubleSolenoid.Value.kReverse) ? elevatorConstantSpeedCube : elevatorConstantSpeedNoCube;//elevatorMinimumSpeedUp * (height - getCurrentPosition());
	        System.out.println("Equilibrium speed: " + speed);
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
	
	public static void moveToHeight(double targetHeight, double currentHeight, double distanceToLift) {
		int direction = (currentHeight > targetHeight) ? -1 : 1;

		double distanceRemaining = Math.abs(currentHeight - targetHeight);
		double fractionRemaining = distanceRemaining/distanceToLift;
		double scaledFraction = fractionRemaining * 4;
		
		double fractionLifted = 1 - fractionRemaining;
		double scaledFractionLifted = fractionLifted * 5;
		
		if (scaledFraction > 1) {
			if (fractionLifted < 0.2) {
				scaledFraction = scaledFractionLifted;
			} else {
				scaledFraction = 1;
			}
		} /*else if (scaledFraction < elevatorMinimumSpeedUp && direction == 1) {
			scaledFraction = elevatorMinimumSpeedUp;
		} else if (scaledFraction < elevatorMinimumSpeedDown && direction == -1) {
			scaledFraction = elevatorMinimumSpeedDown;
		}*/
		
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
		double scaledFraction = fractionRemaining * 4;
		
		double fractionLifted = 1 - fractionRemaining;
        double scaledFractionLifted = fractionLifted * 5;
		
		if (scaledFraction > 1) {
            if (fractionLifted < 0.2) {
                scaledFraction = scaledFractionLifted;
            } else {
                scaledFraction = 1;
            }
        }
		Elevator.set(direction * scaledFraction);
		return false;
	}
	
	public static boolean moveToFloorAuto(double totalDistanceToLift) {
		double currentHeight = getCurrentPosition();
		int direction = -1;
		
		double distanceRemaining = Math.abs(currentHeight - floorHeight);
		if (Elevator.isAtFloor()) {
			encoder.reset();
			Elevator.stop();
			return true;
		}
		
		double fractionRemaining = Math.abs(distanceRemaining/totalDistanceToLift);
		double scaledFraction = fractionRemaining * 4;
		
		double fractionLifted = 1 - fractionRemaining;
        double scaledFractionLifted = fractionLifted * 5;
		
        if (fractionLifted < 0.2) {
        	scaledFraction = scaledFractionLifted;
        } else if (getCurrentPosition() < floorHeight + elevatorDecelerationDistance) {
        	scaledFraction = (getCurrentPosition() - floorHeight)/elevatorDecelerationDistance;
        }
		
		Elevator.set(direction * scaledFraction);
		return false;
		
	}
	
	
	public static enum ElevatorState {
		MANUAL, FLOOR, SWITCH, SCALE;
	}
	
	/*
	public static void setHeight(double height) {
		if (height < floorHeight) {
		} else if (height > maximumHeight) {
			height = maximumHeight;
		}
		double absoluteHeight = height - floorHeight;
		double ticks = convertVerticalInchesToTicks(absoluteHeight);
		//pid.setSetpoint(ticks);
	}*/
	
	/*
	public Elevator() {
		super("Elevator", kP_Lift, kI_Lift, kD_Lift);
		PIDController pid = getPIDController();
		
		pid.enable();
		//pid.setOutputRange(-1, 1);
		pid.setInputRange(0, (maximumHeight - floorHeight) * (1/verticalInchesPerTick));
		pid.setAbsoluteTolerance(convertVerticalInchesToTicks(0.5));
		pid.setContinuous(false);
		
		motor1 = new PIDMotor(spark1, elevatorRampBand, pid);
		motor2 = new PIDMotor(spark1, elevatorRampBand, pid);

	}*/
	
	
	/*
	public static double convertVerticalInchesToTicks(double inches) {
		double rotations = Math.floor(encoder.get()/1024);
		double ticksPerInch = (1/verticalInchesPerTick) + (1/(rotations * Math.PI * 2 * ropeThickness));
		return inches * ticksPerInch;
	}*/
	
}