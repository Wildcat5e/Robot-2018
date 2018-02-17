package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import static org.usfirst.frc.team6705.robot.Constants.*;

public class Intake {

	static Spark rollersLeft = new Spark(leftIntakeSparkChannel);
	static Spark rollersRight = new Spark(rightIntakeSparkChannel);
	
	static DoubleSolenoid leftSolenoid; 
	static DoubleSolenoid rightSolenoid; 

	public static void setup() {
		leftSolenoid = new DoubleSolenoid(intakeSolenoidA, intakeSolenoidB);
		rightSolenoid = new DoubleSolenoid(intakeSolenoidA, intakeSolenoidB);
	}
	
	public static void intake() {
		rollersLeft.set(rollersSpeed);
		rollersRight.set(-rollersSpeed);
	}
	
	public static void outtake() {
		rollersLeft.set(-rollersSpeed);
		rollersRight.set(rollersSpeed);
	}
	
	public static boolean outtakeForTime(double time, double startTime) {
		if (Robot.timer.get() - startTime >= time) {
			return true;
		}
		outtake();
		return false;
	}
	
	public static boolean intakeForTime(double time, double startTime) {
		if (Robot.timer.get() - startTime >= time) {
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
		leftSolenoid.set(DoubleSolenoid.Value.kForward);
		rightSolenoid.set(DoubleSolenoid.Value.kForward);
		
	}
	
	
	public static void close() {
		leftSolenoid.set(DoubleSolenoid.Value.kReverse);
		rightSolenoid.set(DoubleSolenoid.Value.kReverse);
		
	}
	
	public static enum IntakeState {
		MANUAL, INTAKING, OUTTAKING;
	}
	
	
}

