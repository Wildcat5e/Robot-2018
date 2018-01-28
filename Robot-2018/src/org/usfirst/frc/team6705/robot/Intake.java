package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import static org.usfirst.frc.team6705.robot.Constants.*;

public class Intake {

	static Spark rollersLeft = new Spark(leftIntakeSparkChannel);
	static Spark rollersRight = new Spark(rightIntakeSparkChannel);
	
	static DoubleSolenoid leftSolenoid = new DoubleSolenoid(leftIntakeSolenoidForward, leftIntakeSolenoidReverse);
	static DoubleSolenoid rightSolenoid = new DoubleSolenoid(rightIntakeSolenoidForward, rightIntakeSolenoidReverse);

	public static void setup() {
		rollersLeft.setSafetyEnabled(false);
		rollersRight.setSafetyEnabled(false);
	}
	
	public static void intakeCube() {
		
		rollersLeft.set(rollersSpeed);
		rollersRight.set(-rollersSpeed);
		
	}
	
	public static void outtakeCube() {
		
		rollersLeft.set(-rollersSpeed);
		rollersRight.set(rollersSpeed);
		
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
	
	
}

