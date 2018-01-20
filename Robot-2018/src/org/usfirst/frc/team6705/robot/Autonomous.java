package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;


public class Autonomous {
	
	static public void baselineAuto() {
		boolean operating = true;
		
		if (operating) {
			DriveTrain.moveByDistance(132); //Move forward 11 feet = 132 inches
			operating = false; //Change operating to false; now done with auto routine
		} else {
			DriveTrain.stop(); //Stop moving
		}
		
	}
	
	static public void leftSwitchAuto(String startingPosition) {
		switch (startingPosition) {
			case left:
				//Code for starting on left
				break;
			case middle:
				//Code for starting in middle; most likely scenario
				break;
			case right:
				//Code for starting on right
				break;
		}
	}
	
	static public void rightSwitchAuto(String startingPosition) {
		switch (startingPosition) {
			case left:
				//Code for starting on left
				break;
			case middle:
				//Code for starting in middle; most likely scenario
				break;
			case right:
				//Code for starting on right
				break;
		}
	}
	
	static public void leftScaleAuto(String startingPosition) {
			switch (startingPosition) {
			case left:
				//Code for starting on left
				break;
			case middle:
				//Code for starting in middle; most likely scenario
				break;
			case right:
				//Code for starting on right
				break;
			}
	}

	static public void rightScaleAuto(String startingPosition) {
		switch (startingPosition) {
			case left:
				//Code for starting on left
				break;
			case middle:
				//Code for starting in middle; most likely scenario
				break;
			case right:
				//Code for starting on right
				break;
		}	
	}
	
	
	
	
}
