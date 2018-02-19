package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Autonomous {
	public int state = 0;
	private static double previousTime = 0;
	private static double previousElevatorHeight = floorHeight;
	private static boolean isLifting = false;
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
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
	}
	
	private void endAuto() {
		DriveTrain.stop();
		Intake.stopRollers();
		Elevator.stop();
		System.out.println("Finished auto routine!");
	}
	
	public void resetAuto() {
		state = 0;
		previousTime = 0;
		previousElevatorHeight = floorHeight;
	}
	
	private int nextState(int current) {
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
		previousTime = Robot.timer.get();
		previousElevatorHeight = Elevator.getCurrentPosition();
		SmartDashboard.putNumber("Auto State", current + 1);
		DriveTrain.stop();
		return current + 1;
	}
	
	//***************************************************************************//
	
	public void basicAuto() {
		System.out.println("Running basic auto");
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.moveByDistance(60, velocitySlow)) {
				state = nextState(state);
			}
			break;
		case 2:
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//

	public void testAuto() {
		
		
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.moveByDistance(120, velocityMedium)) {
				state = nextState(state);
			}
			break;
		case 2:
			if (DriveTrain.turnDegrees(90)) {
				state = nextState(state);
			}
			break;
		case 3:
			if (DriveTrain.moveByDistance(120, velocityMedium)) {
				state = nextState(state);
			}
			break;
		case 4:
			if (DriveTrain.turnDegrees(90)) {
				state = nextState(state);
			}
			break;
		case 5:
			if (DriveTrain.moveByDistance(120, velocityMedium)) {
				state = nextState(state);
			}
			break;
		case 6:
			if(DriveTrain.turnDegrees(90)) {
				state = nextState(state);
			}
			break;
		case 7:
			if (DriveTrain.moveByDistance(120, velocityMedium)) {
				state = nextState(state);
			}
			break;     
		case 8:
			endAuto();
			break;
		}
	}
	//****************************************************************************//
	public void testStallAuto() {
        switch (state) {
        case 0:
            setupAuto();
            state = nextState(state);
            break;
        case 1:
            if (DriveTrain.moveTillStall()) {
                state = nextState(state);
            }
            break;
        case 2:
        	System.out.println("finished auto");
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
			if (DriveTrain.moveByDistance(132, velocityMedium)) {
				state = nextState(state);
			}
			break;
		case 2:
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//
	
	public void switchAuto(String startingPosition, int switchSide, int scaleSide) {
		//SwitchSide: 1 -> left, -1 -> right
		double horizDistance = (switchSide == 1) ? 57.5 : 54.5;
		
		
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if ((startingPosition == left && switchSide == 1) || (startingPosition == right && switchSide == -1)) { //Same side switch auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward until parallel with switch
				isLifting = true;
				if (DriveTrain.moveByDistance(148.5, velocityMedium) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Move until runs into switch wall
				if (DriveTrain.moveByDistance(20, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (switchSide != scaleSide) {
					if (DriveTrain.moveByDistance(-20, velocitySlow)) {
						state = nextState(state);
					}
				} else {
					endAuto();				
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(40, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 9:
				isLifting = true;
				//Elevator.setHeight(floorHeight);
				if (DriveTrain.moveByDistance(200, velocityFast) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 11:
				Intake.open();
				if (DriveTrain.moveByDistance(50, velocitySlow)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 12:
				isLifting = true;
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(-70, velocityMedium) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 13:
				if (DriveTrain.turnDegrees(180)) {
					state = nextState(state);
				}
				break;
			case 14:
				if (DriveTrain.moveByDistance(20, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 15:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 16:
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
				if (DriveTrain.moveByDistance(50, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Move across field in correct direction
				if (DriveTrain.moveByDistance(horizDistance, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn & raise elevator
				//Elevator.setHeight(switchHeight);
				if (DriveTrain.turnDegrees(-90 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 5: //Move elevator
				isLifting = true;
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 6: //Move forward rest of distance
				if (DriveTrain.moveByDistance(51, velocitySlow)) {
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
	
	public void twoCubeAuto(String startingPosition, int scaleSide, int switchSide) {
		if ((startingPosition == left && scaleSide == 1) || (startingPosition == right && scaleSide == -1) ||
				(startingPosition == left && scaleSide == -1 && switchSide == -1) || (startingPosition == right && scaleSide == 1 && switchSide == 1)) {
			scaleAuto(startingPosition, scaleSide, switchSide);
		} else {
			switchAuto(startingPosition, switchSide, scaleSide);
		}
	}
	
	//***************************************************************************//
	
	public void scaleAuto(String startingPosition, int scaleSide, int switchSide) {
		//scaleSide: 1 -> left, -1 -> right
		
		
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if ((startingPosition == left && scaleSide == 1) || (startingPosition == right && scaleSide == -1)) { //Same side scale auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward part way
				if (DriveTrain.moveByDistance(135.25, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 2: //Move at angle and lift elevator
				isLifting = true;
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(131, 12 * -scaleSide, velocityMedium) &&  Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 3: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn Around
				if (switchSide == scaleSide) {
					if (DriveTrain.turnDegrees(-scaleSide * (175 - 12))) {
						state = nextState(state);	
					}
				} else {
					if (DriveTrain.turnDegrees(-scaleSide * (122 - 12))) {
						state = nextState(state);
					}
				}
				break;
			case 5: //Outtake
				if (switchSide == scaleSide) { //Switch on same side
					Intake.open();
					isLifting = true;
					//Elevator.setHeight(floorHeight);
					if (DriveTrain.moveByDistance(50, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
						isLifting = false;
						Intake.close();
						state = nextState(state);
					}
				} else {  //Switch on opposite side
					isLifting = true;
					//Elevator.setHeight(floorHeight);
					if (DriveTrain.moveByDistance(150, velocityMedium) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
						isLifting = false;
						state = nextState(state);
					}
				}
				break;
			case 6:
				if (switchSide == scaleSide) {
					if (Intake.intakeForTime(timeToRollIn, previousTime)) {
						state = nextState(state);
					}
				} else {
					if (DriveTrain.turnDegrees(-scaleSide * (180 - (122 - 12)))) {
						state = nextState(state);
					}
				}
				break;
			case 7:
				if (switchSide == scaleSide) {
					isLifting = true;
					//Elevator.setHeight(switchHeight);
					if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight)) {
						isLifting = false;
						state = nextState(state);
					}
				} else {
					Intake.open();
					if (DriveTrain.moveByDistance(30, velocitySlow)) {
						Intake.close();
						state = nextState(state);
					}
				}
				break;
			case 8:
				if (switchSide == scaleSide) {
					if (DriveTrain.moveByDistance(13, velocitySlow)) {
						state = nextState(state);
					}
				} else {
					if (Intake.intakeForTime(timeToRollIn, previousTime)) {
						state = nextState(state);
					}
				}
				break;
			case 9:
				if (switchSide == scaleSide) {
					if (Intake.outtakeForTime(timeToRollOut, previousTime))  {
						state = nextState(state);
					}
				} else {
					isLifting = true;
					//Elevator.setHeight(switchHeight);
					if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight)) {
						isLifting = false;
						state = nextState(state);
					}
				}
				break;
			case 10:
				if (switchSide == scaleSide) {
					state = nextState(state);
				} else {
					if (DriveTrain.moveByDistance(13, velocitySlow)) {
						state = nextState(state);
					}
				}
				break;
			case 11:
				if (switchSide == scaleSide) {
					state = nextState(state);
				} else {
					if (Intake.outtakeForTime(timeToRollOut, previousTime))  {
						state = nextState(state);
					}
				}
				break;
			case 12:
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
				if (DriveTrain.moveByDistance(218, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(-90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Drive across the field
				if (DriveTrain.moveByDistance(200, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn
				if (DriveTrain.turnDegrees(90 * scaleSide)) {
					state = nextState(state);
				}
				break;
			case 5: //Drive rest of distance to scale
				isLifting = true;
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(55, velocityMedium) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 6: //Move elevator
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break; 
			case 7: //Turn
				if (DriveTrain.turnDegrees(175 * -scaleSide)) {
					state = nextState(state);
				}
				break;
			case 8: //Inch forward under scale
				Intake.open();
				isLifting = true;
				//Elevator.setHeight(floorHeight);
				if (DriveTrain.moveByDistance(50, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					isLifting = false;
					Intake.close();
					state = nextState(state);
				}
				break;
			case 9: //Outtake
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 10:
				isLifting = true;
				//Elevator.setHeight(switchHeight);
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight)) {
					isLifting = false;
					state = nextState(state);
				}
				break;
			case 11:
				if (DriveTrain.moveByDistance(13, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 12:
				if (Intake.outtakeForTime(timeToRollIn, previousTime))  {
					state = nextState(state);
				}
				break;
			case 13:
				endAuto();
				break;
			}
		} else {
			//Stupid, never going to go for scale if start in middle
			SmartDashboard.putNumber("Auto State", -1);
		}
	}
		
}