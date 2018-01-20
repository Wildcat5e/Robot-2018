package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;

public class Intake {

	static Spark rollersLeft = new Spark(Constants.leftIntakeSpark);
	static Spark rollersRight = new Spark(Constants.rightIntakeSpark);
	
	static DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.leftIntakeSolenoidForward, Constants.leftIntakeSolenoidReverse);
	static DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.rightIntakeSolenoidForward, Constants.rightIntakeSolenoidReverse);

	
	public static void intakeCube() {
		
		rollersLeft.set(Constants.rollersSpeed);
		rollersRight.set(-Constants.rollersSpeed);
		
	}
	
	public static void outtakeCube() {
		
		rollersLeft.set(-Constants.rollersSpeed);
		rollersRight.set(Constants.rollersSpeed);
		
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

