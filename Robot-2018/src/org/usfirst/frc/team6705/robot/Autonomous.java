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
	 *        Distance from front of robot to front of switch = 107
	 *        Distance from middle of robot to middle of switch = 151.5
	 *        Distance from middle of robot to middle of scale = 307.5
	 *        
	 */
	
	static public void leftSwitchAuto(String startingPosition) {
		if (operating) {
			switch (startingPosition) {
				case left:
					//Code for starting on left
					DriveTrain.moveByDistance(151.5);
					DriveTrain.turnDegrees(-90);
					//LIFT CUBE TO SWITCH HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					//Code for starting in middle; most likely scenario
					DriveTrain.moveByDistance(53);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(54);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(72);
					//LIFT CUBE TO SWITCH HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					//Code for starting on right
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(90);
					//LIFT CUBE TO SWITCH HEIGHT
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
					//Code for starting on left
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(-90);
					//LIFT CUBE TO SWITCH HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					//Code for starting in middle; most likely scenario
					DriveTrain.moveByDistance(53);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(54);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(72);
					//LIFT CUBE TO SWITCH HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					//Code for starting on right
					DriveTrain.moveByDistance(151.5);
					DriveTrain.turnDegrees(90);
					//LIFT CUBE TO SWITCH HEIGHT
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
				//Code for starting on left
				DriveTrain.moveByDistance(324);
				DriveTrain.turnDegrees(-90);
				//LIFT CUBE TO SCALE HEIGHT
				Intake.outtakeCube();
				operating = false;
				break;
			case middle:
				//Code for starting in middle
				DriveTrain.moveByDistance(48);
				DriveTrain.turnDegrees(90);
				DriveTrain.moveByDistance(168);
				DriveTrain.turnDegrees(-90);
				DriveTrain.moveByDistance(276);
				DriveTrain.turnDegrees(-90);
				//LIFT CUBE TO SCALE HEIGHT
				Intake.outtakeCube();
				operating = false;
				break;
			case right:
				//Code for starting on right
				DriveTrain.moveByDistance(240);
				DriveTrain.turnDegrees(90);
				DriveTrain.moveByDistance(121);
				DriveTrain.turnDegrees(-90);
				DriveTrain.moveByDistance(42);
				//LIFT CUBE TO SCALE HEIGHT
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
					//Code for starting on left
					DriveTrain.moveByDistance(240);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(121);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(42);
					//LIFT CUBE TO SCALE HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case middle:
					//Code for starting in middle
					DriveTrain.moveByDistance(48);
					DriveTrain.turnDegrees(-90);
					DriveTrain.moveByDistance(168);
					DriveTrain.turnDegrees(90);
					DriveTrain.moveByDistance(276);
					DriveTrain.turnDegrees(90);
					//LIFT CUBE TO SCALE HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				case right:
					//Code for starting on right
					DriveTrain.moveByDistance(324);
					DriveTrain.turnDegrees(90);
					//LIFT CUBE TO SCALE HEIGHT
					Intake.outtakeCube();
					operating = false;
					break;
				}
			} else {
				DriveTrain.stop();
			}
	}
	
	
	
	
}
