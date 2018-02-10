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
	 *        Middle - line up touching edge to exchange zone
	 *        Left & Right - line up robot at corner diagonal of field
	 *        
	 */
	
	private void setupAuto() {
//		System.out.println("Setting up auto");
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
	}
	
	private void endAuto() {
		DriveTrain.stop();
		Intake.stopRollers();
		Elevator.stop();
		System.out.println("Finished auto routine!");
	}
	
	private int nextState(int current) {
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
		previousTime = Robot.timer.get();
		SmartDashboard.putNumber("Auto State", current + 1);
		DriveTrain.stop();
		return current + 1;
	}
	
	//***************************************************************************//
	
	public void basicAuto() {
		System.out.println("Running basic auto");
		DriveTrain.setVelocity(6000, 6000);
	}
	
	//***************************************************************************//

	public void testAuto() {
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.moveByDistance(83, velocitySlow)) {
				state = nextState(state);
			}
			break;
		case 2:
			if (DriveTrain.turnDegrees(-90)) {
				state = nextState(state);
			}
			break;
		case 3:
			if (DriveTrain.moveByDistance(40, velocitySlow)) {
				state = nextState(state);
			}
			break;
		case 4:
			if (DriveTrain.moveByDistance(-40, velocitySlow)) {
				state = nextState(state);
			}
			break;
		case 5:
			if (DriveTrain.turnDegrees(90)) {
				state = nextState(state);
			}
			break;
		case 6:
			if (DriveTrain.moveByDistance(-83, velocitySlow))  {
				state = nextState(state);
			}
			break;
		case 7:
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//
	
	public void baselineAuto() {
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.moveByDistance(132, velocityFast)) {
				state = nextState(state);
			}
			break;
		case 2:
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//
	
	public void switchAuto(String startingPosition, int switchSide) {
		//SwitchSide: 1 -> left, -1 -> right
		double horizDistance = (switchSide == 1) ? 59 : 49;
		
		if ((startingPosition == left && switchSide == 1) || (startingPosition == right && switchSide == -1)) { //Same side switch auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward until parallel with switch
				if (DriveTrain.moveByDistance(148.5, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Move elevator
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 4: //Move until runs into switch wall
				if (DriveTrain.moveTillStall()) {
					state = nextState(state);
				}
				break;
			case 5: //Outtake
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
			case 1: //Move forward halfway
				if (DriveTrain.moveByDistance(50, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Move across field in correct direction
				if (DriveTrain.moveByDistance(horizDistance, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 5: //Move elevator
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 6: //Move forward rest of distance
				if (DriveTrain.moveByDistance(51, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 7: //Outtake
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
			baselineAuto();
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
			case 1: //Move forward to scale
				if (DriveTrain.moveByDistance(304.5, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Move elevator
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 3: //Turn
				if (DriveTrain.turnDegrees(-90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 4: //Inch forward under scale
				if (DriveTrain.moveByDistance(5, velocitySlow)) {
					state = nextState(state);
				}
			case 5: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				endAuto();
				break;
			}

		} else if ((startingPosition == left && scaleSide == -1) || (startingPosition == right && scaleSide == 1)) { //Opposite side scale auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Drive forward past switch
				if (DriveTrain.moveByDistance(209.235, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(-90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Drive across the field
				if (DriveTrain.moveByDistance(230, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn
				if (DriveTrain.turnDegrees(90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 5: //Drive rest of distance to scale
				if (DriveTrain.moveByDistance(95.265, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 6: //Move elevator
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight)) {
					state = nextState(state);
				}
				break; 
			case 7: //Turn
				if (DriveTrain.turnDegrees(90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 8: //Inch forward under scale
				if (DriveTrain.moveByDistance(5, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 9: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 10:
				endAuto();
				break;
			}
		} else {
			//Stupid, never going to go for scale if start in middle
			SmartDashboard.putNumber("Auto State", -1);
		}
	}
		
}
