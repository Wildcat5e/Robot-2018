package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

//import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.MotionProfileDataSets.*;


public class Autonomous {
	public int state = 0;
	private double previousTime = 0;
	private double previousElevatorHeight = floorHeight;
	public boolean isLifting = false;
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
		//Elevator.stop();
		DriveTrain.leftTalon.set(ControlMode.PercentOutput, 0);
		DriveTrain.rightTalon.set(ControlMode.PercentOutput, 0);
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
	
	MotionProfile exampleProfile = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void motionProfileTestAuto() {
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		switch (state) {
		case 0:
			setupAuto();
			DriveTrain.startMotionProfile(exampleProfile);
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.runMotionProfile(exampleProfile) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
				state = nextState(state);
			}
			break;
		case 2:
			if (DriveTrain.moveByDistance(-40, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
				state = nextState(state);
			}
		case 3:
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
	
	MotionProfile profileSwitchLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileSwitchRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void switchAuto(String startingPosition, int switchSide, int scaleSide) {
		//SwitchSide: 1 -> left, -1 -> right
		//double horizDistance = (switchSide == 1) ? 57.5 : 54.5;
		
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (startingPosition == middle && switchSide == 1) { //Left switch auto 
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileSwitchLeft);
				DriveTrain.startMotionProfile(profileSwitchLeft);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileSwitchLeft) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(-30, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 5:
				endAuto();
				break;
			}
		} else if (startingPosition == middle && switchSide == -1) { //Right switch auto
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileSwitchRight);
				DriveTrain.startMotionProfile(profileSwitchRight);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileSwitchRight) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(-30, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 5:
				endAuto();
				break;
			}
		} else {
			SmartDashboard.putNumber("Auto State", -1);
			System.out.println("DON'T DO SWITCH AUTO ON THE SIDES STUPID");
		}
		
		/*
		if ((startingPosition == left && switchSide == 1) || (startingPosition == right && switchSide == -1)) { //Same side switch auto
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward until parallel with switch
				if (DriveTrain.moveByDistance(148.5, velocityMedium) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
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
				//Elevator.setHeight(floorHeight);
				if (DriveTrain.moveByDistance(200, velocityFast) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
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
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(-70, velocityMedium) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
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
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
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
		}*/
	}
	
	//***************************************************************************//
	
	MotionProfile profileScaleLeft_Same = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileScaleLeft_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileGetCubeFromScaleRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileGetCubeFromScaleLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileCubeToScaleRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileCubeToScaleLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void doubleScaleAutoLeft(int scaleSide) {
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (scaleSide == 1) { //same side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Same);
				DriveTrain.startMotionProfile(profileScaleLeft_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 3:
				if (DriveTrain.turnDegrees(180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					DriveTrain.setupMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(180) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.runMotionProfile(profileCubeToScaleLeft)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 9:
				endAuto();
			}
			
		} else if (scaleSide == -1) {//opposite side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Opposite);
				DriveTrain.startMotionProfile(profileScaleLeft_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Opposite) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 3:
				if (DriveTrain.turnDegrees(-180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					DriveTrain.setupMotionProfile(profileCubeToScaleRight);
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(-180) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleRight);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.runMotionProfile(profileCubeToScaleRight)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 9:
				endAuto();
			}
			
		}
	}
	
	//***************************************************************************//

	MotionProfile profileScaleRight_Same = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileScaleRight_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	public void doubleScaleAutoRight(int scaleSide) {
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (scaleSide == -1) { //same side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Same);
				DriveTrain.startMotionProfile(profileScaleRight_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 3:
				if (DriveTrain.turnDegrees(-180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					DriveTrain.setupMotionProfile(profileCubeToScaleRight);
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(-180) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleRight);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.runMotionProfile(profileCubeToScaleRight)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 9:
				endAuto();
			}
			
		} else if (scaleSide == 1) {//opposite side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Opposite);
				DriveTrain.startMotionProfile(profileScaleRight_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Opposite) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 3:
				if (DriveTrain.turnDegrees(180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					DriveTrain.setupMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(180) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.runMotionProfile(profileCubeToScaleLeft)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
			case 9:
				endAuto();
			}
			
		}
	}

	//***************************************************************************//
	
	MotionProfile profileGetCubeFromScaleLeft_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileSwitchLeft_Side = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileBackAwayFromSwitchCube_Left = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length, true);
	
	MotionProfile profileCrossFieldToScaleLeftToRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void scaleSwitchAutoLeft(int scaleSide, int switchSide) {
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (scaleSide == 1 && switchSide == 1) {//Same side both scale and switch
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Same);
				DriveTrain.startMotionProfile(profileScaleLeft_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, 10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		} else if (scaleSide == 1 && switchSide == -1) {//Same side scale, opposite side switch
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Same);
				DriveTrain.startMotionProfile(profileScaleLeft_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft_Opposite);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft_Opposite);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft_Opposite) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, -10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
			
		} else if (scaleSide == -1 && switchSide == -1) {//Opposite side both scale and switch
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Opposite);
				DriveTrain.startMotionProfile(profileScaleLeft_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Opposite) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(-180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, -10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
			
		} else if (scaleSide == -1 && switchSide == 1) {//Opposite side scale, same side switch
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileSwitchLeft_Side);
				DriveTrain.startMotionProfile(profileSwitchLeft_Side);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileSwitchLeft_Side) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileBackAwayFromSwitchCube_Left);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				Intake.open();
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 4:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					DriveTrain.startMotionProfile(profileBackAwayFromSwitchCube_Left);
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.runMotionProfile(profileBackAwayFromSwitchCube_Left)) {
					DriveTrain.setupMotionProfile(profileCrossFieldToScaleLeftToRight);
					DriveTrain.startMotionProfile(profileCrossFieldToScaleLeftToRight);
					state = nextState(state);
				}
			case 6:
				if (DriveTrain.runMotionProfile(profileCrossFieldToScaleLeftToRight) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-20, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		}
		
	}
	
	//***************************************************************************//

	MotionProfile profileGetCubeFromScaleRight_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	MotionProfile profileSwitchRight_Side = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileBackAwayFromSwitchCube_Right = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length, true);
	
	MotionProfile profileCrossFieldToScaleRightToLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void scaleSwitchAutoRight(int scaleSide, int switchSide) {
		if (!isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (scaleSide == -1 && switchSide == -1) {//Same side both scale and switch
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Same);
				DriveTrain.startMotionProfile(profileScaleRight_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(-180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, 10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		} else if (scaleSide == -1 && switchSide == 1) {//Same side scale, opposite side switch
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Same);
				DriveTrain.startMotionProfile(profileScaleRight_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight_Opposite);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(-180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight_Opposite);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight_Opposite) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, -10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
			
		} else if (scaleSide == 1 && switchSide == 1) {//Opposite side both scale and switch
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Opposite);
				DriveTrain.startMotionProfile(profileScaleRight_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Opposite) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(180)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 5:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
			case 6:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(12, velocitySlow, -10)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
			
		} else if (scaleSide == 1 && switchSide == -1) {//Opposite side scale, same side switch
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileSwitchRight_Side);
				DriveTrain.startMotionProfile(profileSwitchRight_Side);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileSwitchRight_Side) && Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileBackAwayFromSwitchCube_Right);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 3:
				Intake.open();
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 4:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					DriveTrain.startMotionProfile(profileBackAwayFromSwitchCube_Right);
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.runMotionProfile(profileBackAwayFromSwitchCube_Right)) {
					DriveTrain.setupMotionProfile(profileCrossFieldToScaleRightToLeft);
					DriveTrain.startMotionProfile(profileCrossFieldToScaleRightToLeft);
					state = nextState(state);
				}
			case 6:
				if (DriveTrain.runMotionProfile(profileCrossFieldToScaleRightToLeft) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-20, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		}
		
	}
	
	//***************************************************************************//
	/*
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
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(131, 12 * -scaleSide, velocityMedium) &&  Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
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
					//Elevator.setHeight(floorHeight);
					if (DriveTrain.moveByDistance(50, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
						Intake.close();
						state = nextState(state);
					}
				} else {  //Switch on opposite side
					//Elevator.setHeight(floorHeight);
					if (DriveTrain.moveByDistance(150, velocityMedium) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
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
					//Elevator.setHeight(switchHeight);
					if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
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
					//Elevator.setHeight(switchHeight);
					if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
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
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(55, velocityMedium) && Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
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
				//Elevator.setHeight(floorHeight);
				if (DriveTrain.moveByDistance(50, velocitySlow) && Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
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
				//Elevator.setHeight(switchHeight);
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
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
	}*/
		
}