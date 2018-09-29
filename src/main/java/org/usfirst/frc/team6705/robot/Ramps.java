package org.usfirst.frc.team6705.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static org.usfirst.frc.team6705.robot.Constants.*;
public class Ramps {

	static DoubleSolenoid leftSolenoid = new DoubleSolenoid(leftRampSolenoidA, leftRampSolenoidB);
	static DoubleSolenoid rightSolenoid = new DoubleSolenoid(rightRampSolenoidA, rightRampSolenoidB);
	
	public static void deploy() {
		leftSolenoid.set(DoubleSolenoid.Value.kForward);
		rightSolenoid.set(DoubleSolenoid.Value.kForward);
	}
}
