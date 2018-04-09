package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import static org.usfirst.frc.team6705.robot.Constants.*;

public class Intake {

	static Spark rollersLeft = new Spark(leftIntakeSparkChannel);
	static Spark rollersRight = new Spark(rightIntakeSparkChannel);
	
	static DoubleSolenoid solenoid = new DoubleSolenoid(intakeSolenoidA, intakeSolenoidB);
	static DoubleSolenoid actuator1 = new DoubleSolenoid(intakeActuatorSolenoidA, intakeActuatorSolenoidB);
	//static DoubleSolenoid actuator2 = new DoubleSolenoid(intakeActuatorSolenoidA, intakeActuatorSolenoidB);
	


	public static void setup() {
	    solenoid.set(DoubleSolenoid.Value.kReverse);
	    rollersRight.setInverted(false);
	    rollersLeft.setInverted(false);
	    
	}
	
	public static void intake() {
		rollersLeft.set(rollersSpeedAuto);
		rollersRight.set(rollersSpeedAuto);
	}
	
	public static void outtake() {
		rollersLeft.set(-rollersSpeedAuto);
		rollersRight.set(-rollersSpeedAuto);
	}
	
	public static void actuateUp() {
		System.out.print("Actuate Intake Up");
		actuator1.set(DoubleSolenoid.Value.kReverse);	
	}
	
	public static void actuateDown() {
		System.out.print("Actuate Intake Down");
		actuator1.set(DoubleSolenoid.Value.kForward);	
	}
	
	public static void roll(double speed) {
	    System.out.println("Rolling speed: " + speed);
		rollersLeft.set(speed * maxRollersSpeed);
		rollersRight.set(speed * maxRollersSpeed);
	}
	
	public static boolean outtakeForTime(double time, double startTime) {
		if (Robot.timer.get() - startTime >= time) {
			stopRollers();
			return true;
		}
		outtake();
		return false;
	}
	
	public static boolean intakeForTime(double time, double startTime) {
		if (Robot.timer.get() - startTime >= time) {
			stopRollers();
			return true;
		}
		intake();
		return false;
	}
	
	public static void stopRollers() {
		
		rollersLeft.set(0);
		rollersRight.set(0);
	}
	
	public static void open() {
	    System.out.print("Open Pneumatics");
		solenoid.set(DoubleSolenoid.Value.kForward);
		
	}
	
	public static void close() {
	    System.out.println("Close Pneumatics");
		solenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	/*
	public static void angleUp() {
		actuator1.set(DoubleSolenoid.Value.kForward);
		actuator2.set(DoubleSolenoid.Value.kOff);
	}
	
	public static void angleDiagonal() {
		actuator1.set(DoubleSolenoid.Value.kOff);
		actuator2.set(DoubleSolenoid.Value.kOff);
	}
	
	public static void angleDown() {
		actuator1.set(DoubleSolenoid.Value.kOff);
		actuator2.set(DoubleSolenoid.Value.kForward);
	}*/
	
	public static enum IntakeState {
		MANUAL, INTAKING, OUTTAKING;
	}
	
	
}

