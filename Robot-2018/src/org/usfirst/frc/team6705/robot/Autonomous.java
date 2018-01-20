package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static org.usfirst.frc.team6705.robot.Constants.*;


public class Autonomous {

	static public void driveForward(DifferentialDrive drive, double speed) {
		drive.tankDrive(speed, speed);
	}
	
	static public void stop(DifferentialDrive drive) {
		drive.tankDrive(0, 0);
	}
	
	
	
}
