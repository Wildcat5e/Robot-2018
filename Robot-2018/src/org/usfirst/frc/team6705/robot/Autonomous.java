package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Autonomous {
	private static int state = 0;
	private static double previousTime = 0;
	/*
	 * 	Note: Distance from alliance wall to front of switch = 140 in
	 *        Distance from front of robot to front of switch = 101 in
	 *        Distance from middle of robot to middle of switch = 148.5
	 *        Distance from middle of robot to middle of scale = 304.5
	 *        Left & Ride - line up robot at corner diagonal of field
	 *        
	 */
	
	private void setupAuto() {
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
	}
	
	private void endAuto() {
		DriveTrain.stop();
		Intake.stopRollers();
		Elevator.stop();
	}
	
	private int nextState(int current) {
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
		previousTime = Robot.timer.get();
		SmartDashboard.putNumber("Auto State", current + 1);
		return current++;
	}
	
	//***************************************************************************//
	
	public void baselineAuto() {
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
		case 1:
			if (DriveTrain.moveByDistance(132, velocityMedium)) {
				state = nextState(state);
			}
		case 2:
			endAuto();
		}
	}
	
	//***************************************************************************//
	
	public void switchAuto(String startingPosition, int switchSide) {
		//SwitchSide: 1 -> left, -1 -> right
		
		if ((startingPosition == left && switchSide == 1) || (startingPosition == right && switchSide == -1)) { //Same side switch auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.moveByDistance(148.5, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.moveTillStall()) {
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				endAuto();
				break;
			}
			
		} else if (startingPosition == middle) {
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.moveByDistance(50, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.moveByDistance(54, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.moveByDistance(51, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				endAuto();
				break;
			} 
		} else {
			//Stupid, never going to do left switch if starting on the right & vice versa
			SmartDashboard.putNumber("Auto State", -1);
		}
	}
	
	//***************************************************************************//
	
	public void scaleAuto(String startingPosition, int scaleSide) {
		//scaleSide: 1 -> left, -1 -> right
		
		if ((startingPosition == left && scaleSide == 1) || (startingPosition == right && scaleSide == -1)) { //Same side scale auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.moveByDistance(304.5, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(-90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			//Maybe drive forward a little depending on dimensions to get cube in scale
			case 4:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 5:
				endAuto();
				break;
			}

		} else if ((startingPosition == left && scaleSide == -1) || (startingPosition == right && scaleSide == 1)) { //Opposite side scale auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.moveByDistance(48, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.turnDegrees(-90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(121, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.moveByDistance(42, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				endAuto();
				break;
			}
		} else {
			//Stupid, never going to go for scale if start in middle
			SmartDashboard.putNumber("Auto State", -1);
		}
	}
		
}
