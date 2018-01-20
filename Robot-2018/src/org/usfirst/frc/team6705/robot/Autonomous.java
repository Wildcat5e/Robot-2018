package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Autonomous.BaselineAutoState;


public class Autonomous {

	static public void driveForward(DifferentialDrive drive, double speed, double leftMinusRightDifference) {
		if (leftMinusRightDifference > 1) {
			drive.tankDrive(speed - 0.5, speed);
		} else if (leftMinusRightDifference < -1) {
			
		} else {
			drive.tankDrive(speed, speed);
		}
	}
	
	static public void stop(DifferentialDrive drive) {
		drive.tankDrive(0, 0);
	}
	
	enum BaselineAutoState {
		DRIVEFORWARD, WAIT
	}
	
	static public void baselineAuto() {
		BaselineAutoState state = BaselineAutoState.DRIVEFORWARD;
		
		switch (state) {
			case DRIVEFORWARD:
				DriveTrain.moveByDistance(132);
				state = BaselineAutoState.WAIT;
				break;
			case WAIT:
				DriveTrain.stop();
				break;
		}
	}
	
	static public void leftSwitchAuto() {
		
	}
	
	static public void rightSwitchAuto() {
		
	}
	
	static public void leftScaleAuto() {
		
	}

	static public void rightScaleAuto() {
	
	}
	
	
	
	
}
