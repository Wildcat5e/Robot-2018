package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;


public class Autonomous {
	static boolean operating = true;
	
	static public void baselineAuto() {
		if (operating) {
			DriveTrain.moveByDistance(132); //Move forward 11 feet = 132 inches
			operating = false; //Change operating to false; now done with auto routine
		} else {
			DriveTrain.stop(); //Stop moving
		}
	}
	
	/*
	 * 	Note: Distance from alliance wall to front of switch = 140 in
	 *        Distance from front of robot to front of switch = 101 in
	 *        Distance from middle of robot to middle of switch = 148.5
	 *        Distance from middle of robot to middle of scale = 304.5
	 *        
	 */
	
	static public void leftSwitchAuto(String startingPosition) {
		if (operating) {
			switch (startingPosition) {
				case left:
					DriveTrain.moveByDistance(148.5);
					DriveTrain.turnDegrees(-90);
					Elevator.toSwitch();
					DriveTrain.moveTillStall();
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					Elevator.toSwitch();
					DriveTrain.moveByDistance(50);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(54);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(51);
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					Elevator.toSwitch();
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(90);
					Intake.outtakeCube();
					operating = false;
					break;
			}
		} else {
			DriveTrain.stop();
		}
	}
	
	static public void rightSwitchAuto(String startingPosition) {
		if (operating) {
			switch (startingPosition) {
				case left:
					Elevator.toSwitch();
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(-90);
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					Elevator.toSwitch();
					DriveTrain.moveByDistance(50);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(54);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(51);
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					DriveTrain.moveByDistance(148.5);
					DriveTrain.turnDegrees(90);
					Elevator.toSwitch();
					DriveTrain.moveTillStall();
					Intake.outtakeCube();
					operating = false;
					break;
			}
		} else {
			DriveTrain.stop();
		}
	}
	
	static public void leftScaleAuto(String startingPosition) {
		if (operating) {	
		switch (startingPosition) {
			case left:
				DriveTrain.moveByDistance(304.5);
				Elevator.toScale();
				DriveTrain.turnDegrees(-90);
				Intake.outtakeCube();
				operating = false;
				break;
			case middle:
				Elevator.toScale();
				DriveTrain.moveByDistance(48);
				DriveTrain.turnDegrees(90);
				DriveTrain.moveByDistance(168);
				DriveTrain.turnDegrees(-90);
				DriveTrain.moveByDistance(276);
				DriveTrain.turnDegrees(-90);
				Intake.outtakeCube();
				operating = false;
				break;
			case right:
				Elevator.toScale();
				DriveTrain.moveByDistance(240);
				DriveTrain.turnDegrees(90);
				DriveTrain.moveByDistance(121);
				DriveTrain.turnDegrees(-90);
				DriveTrain.moveByDistance(42);
				Intake.outtakeCube();
				operating = false;
				break;
			}
		} else {
			DriveTrain.stop();
		}
	}

	static public void rightScaleAuto(String startingPosition) {
		if (operating) {	
			switch (startingPosition) {
				case left:
					Elevator.toScale();
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(42);
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					Elevator.toScale();
					DriveTrain.moveByDistance(48);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(168);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(276);
					DriveTrain.turnDegrees(90);
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					DriveTrain.moveByDistance(304.5);
					Elevator.toScale();
					DriveTrain.turnDegrees(90);
					Intake.outtakeCube();
					operating = false;
					break;
				}
			} else {
				DriveTrain.stop();
			}
	}
	
	
	
	
}
