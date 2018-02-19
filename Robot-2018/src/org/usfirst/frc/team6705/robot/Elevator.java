package org.usfirst.frc.team6705.robot;
import static org.usfirst.frc.team6705.robot.Constants.*;
import org.usfirst.frc.team6705.robot.PIDMotor;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class Elevator /*extends PIDSubsystem*/ {
	
	static Spark spark1 = new Spark(elevatorSpark1);
	static Spark spark2 = new Spark(elevatorSpark2);
	
	static SpeedControllerGroup motor = new SpeedControllerGroup(spark1, spark2);

	static Encoder encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
	
	//static PIDController pid = new PIDController(kP_Lift, kI_Lift, kD_Lift, kF_Lift, encoder, motor);
	
	//static PIDMotor motor1;
	//static PIDMotor motor2;

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
	
	
	public static void setup() {
	    //encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false, EncodingType.k4X);
		encoder.reset();
		encoder.setDistancePerPulse(verticalInchesPerTick);
		
		/*
		pid.enable();
		pid.setOutputRange(-0.1, 1);
		pid.setInputRange(0, (maximumHeight - floorHeight) * (1/verticalInchesPerTick));
		pid.setAbsoluteTolerance(convertVerticalInchesToTicks(0.5));
		pid.setContinuous(false);*/
		
	}
	/*
	public void initDefaultCommand() {
    }
	
	 protected double returnPIDInput() {
         return encoder.get(); // returns the encoder value that is providing the feedback for the system
	 }

	 protected void usePIDOutput(double output) {
         motor1.pidWrite(output); // this is where the computed output value from the PIDController is applied to the motor
         motor2.pidWrite(output);
	 }*/
	
	public static double convertTicksToVerticalInches(double ticks) {
		return ticks * verticalInchesPerTick;
		//Implement appropriate function to convert, taking into account the widening/thinning of axle as pulley rolls
	}
	
	public static double convertVerticalInchesToTicks(double inches) {
		return inches * (1/verticalInchesPerTick);
	}
	
	public static double getCurrentPosition() {
		return convertTicksToVerticalInches(encoder.get()) + floorHeight;
	}
	
	public static void setHeight(double height) {
		if (height < floorHeight) {
		} else if (height > maximumHeight) {
			height = maximumHeight;
		}
		double absoluteHeight = height - floorHeight;
		double ticks = convertVerticalInchesToTicks(absoluteHeight);
		//pid.setSetpoint(ticks);
	}
	
	public static void set(double speed) {
	    if ((speed > 0 && getCurrentPosition() < maximumHeight) || (speed < 0 && getCurrentPosition() > floorHeight)) {
	        
            double maxSpeed = (speed < 0) ? elevatorMaxSpeedDown : elevatorMaxSpeedUp;
            System.out.println("Setting lift speed " + speed);
            if (speed < 0) {
                double downSpeed = (speed/2) + 0.55;
                motor.set(downSpeed);
            } else {
                double upSpeed = (speed * (1 - elevatorMinimumSpeedUp)) * + elevatorMinimumSpeedUp;
                motor.set(upSpeed);
            }
        } else {
            stop();
        }
	}
	
	public static void maintainHeight(double height) {
	    if (getCurrentPosition() < height + elevatorTolerance) {
	        System.out.println("Trying to maintain height " + height + "Current Position is " + getCurrentPosition());
	        double speed = elevatorConstantSpeed;//elevatorMinimumSpeedUp * (height - getCurrentPosition());
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
	
	
	public static void moveToHeight(double targetHeight, double currentHeight, double distanceToLift) {
		int direction = (currentHeight > targetHeight) ? -1 : 1;

		double distanceRemaining = Math.abs(currentHeight - targetHeight);
		double fractionRemaining = distanceRemaining/distanceToLift;
		double scaledFraction = fractionRemaining * 3;
		
		double fractionLifted = 1 - fractionRemaining;
		double scaledFractionLifted = fractionLifted * 5;
		
		if (scaledFraction > 1) {
			if (fractionLifted < 0.2) {
				scaledFraction = scaledFractionLifted;
			} else {
				scaledFraction = 1;
			}
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
		
		double fractionLifted = 1 - fractionRemaining;
        double scaledFractionLifted = fractionLifted * 5;
		
		if (scaledFraction > 1) {
            if (fractionLifted < 0.2) {
                scaledFraction = scaledFractionLifted;
            } else {
                scaledFraction = 1;
            }
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

