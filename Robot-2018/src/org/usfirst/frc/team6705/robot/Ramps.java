package org.usfirst.frc.team6705.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static org.usfirst.frc.team6705.robot.Constants.*;
public class Ramps {

	static DoubleSolenoid leftSolenoid = new DoubleSolenoid(leftRampSolenoidOpen, leftRampSolenoidClosed);
	static DoubleSolenoid rightSolenoid = new DoubleSolenoid(rightRampSolenoidOpen, rightRampSolenoidClosed);
	
	public static void deploy() {
		leftSolenoid.set(DoubleSolenoid.Value.kForward);
		rightSolenoid.set(DoubleSolenoid.Value.kForward);
	}
}
