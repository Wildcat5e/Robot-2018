package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

//import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static org.usfirst.frc.team6705.robot.MotionProfileDataSets.*;


public class Autonomous {
	public int state = 0;
	private double previousTime = 0;
	public double previousElevatorHeight = floorHeight;
	public double previousFinalTurningError = 0;
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
		previousFinalTurningError = 0;
		Elevator.hasCompletedLift = false;
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
		previousFinalTurningError = 0;
		isLifting = false;
		Elevator.hasCompletedLift = false;
	}
	
	private int nextState(int current) {
		DriveTrain.resetEncoders();
		DriveTrain.gyro.reset();
		previousTime = Robot.timer.get();
		previousElevatorHeight = Elevator.getCurrentPosition();
		SmartDashboard.putNumber("Auto State", current + 1);
		DriveTrain.stop();
		Elevator.hasCompletedLift = false;
		return current + 1;
	}
	
	//***************************************************************************//
	
	//MotionProfile exampleProfile = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void testAuto() {
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		System.out.println("Test Auto");
		switch (state) {
		case 0:
			setupAuto();
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.moveByDistance(50, velocitySlow) & Elevator.moveToHeightAuto(switchHeight, switchHeight, 1)) {
				state = nextState(state);
			}
			break;
		case 2:
			if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight)) {
				state = nextState(state);
			}
			break;
		/*case 4: 
			if (DriveTrain.moveByDistance(100, velocitySlow) & Elevator.moveToHeightAfterDriving(switchHeight, switchHeight, 1, 50)) {
				state = nextState(state);
			}
			break;*/
		case 3:
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//

	MotionProfile profileStraight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, straightTest_L, straightTest_R, straightTest_R.length);
	
	public void straightMotionProfile() {
		
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		switch (state) {
		case 0:
			setupAuto();
			DriveTrain.setupMotionProfile(profileStraight);
			DriveTrain.startMotionProfile(profileStraight);
			state = nextState(state);
			break;
		case 1:
			if (DriveTrain.runMotionProfile(profileStraight)) {
				state = nextState(state);
			}
			break;
		case 2:
			endAuto();
		}
		
	}
	
	//***************************************************************************//
	
	public void baselineAuto(int scaleSide, String startingPosition) {
		if (!Elevator.isAtFloor() && !isLifting) {
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
			endAuto();
			break;
		}
	}
	
	//***************************************************************************//
	
	public void switchAuto(String startingPosition, int switchSide, int scaleSide) {
		//SwitchSide: 1 -> left, -1 -> right		
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		double diagonalDistance = (switchSide == 1) ? 54 : 47;/*58 : 50;*/
		
		if ((startingPosition == left && switchSide == 1) || (startingPosition == right && switchSide == -1)) { //Same side switch auto
			
		} else if (startingPosition == middle) {
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: 
				if (DriveTrain.moveByDistance(5, velocitySlow) /*&& Elevator.moveToHeightAuto(autoDriveHeight, autoDriveHeight, 1)*/) {
					state = nextState(state);
				}
				break;
			case 2: //Turn
				if (DriveTrain.turnDegrees(45 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 3: //Move across field in correct direction
				if (DriveTrain.moveByDistance(diagonalDistance, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(-45 * switchSide)) {
					state = nextState(state);
				}
				break;
			case 5: //Move elevator
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 6: //Move forward rest of distance
				if (DriveTrain.moveByDistance(29, 0, velocitySlow, 2)) {
					state = nextState(state);
				}
				break;
			case 7: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-29, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 9:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.turnDegrees(switchSide * 45)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (DriveTrain.moveByDistance(-diagonalDistance, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 12:
				if (DriveTrain.turnDegrees(switchSide * -45)) {
					state = nextState(state);
				}
				break;
			case 13:
				endAuto();
				break;
			} 
		} else {
			SmartDashboard.putNumber("Auto State", -1);
		}
	}
	
	//***************************************************************************//

	public void singleScaleAuto(int scaleSide, String startingPosition) {
		
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		System.out.println("Scale side " + scaleSide + "Starting pos " + startingPosition);
		if ((scaleSide == 1 && startingPosition == left) || (scaleSide == -1 && startingPosition == right)) { //Same side
			//Go forward and turn to same side scale
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward part way
				if (DriveTrain.moveByDistance(280, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.turnDegrees(90 * -scaleSide, 15)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.moveByDistance(10, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 5: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);
				}
				break;
			case 8:
				endAuto();
				break;
			}
		} else if ((scaleSide == 1 && startingPosition == right) || (scaleSide == -1 && startingPosition == left)) {//Opposite side
			//Cross field to drop off cube at opposite side scale
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.moveByDistance(190, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.turnDegrees(scaleSide * 90, 6)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(194, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(-scaleSide * 90, 8)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.moveByDistance(50, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-20, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 9:
				if (Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);
				}
				break;
			case 10:
				endAuto();
				break;
			}
		}
	}
	
	//***************************************************************************//
	
	public void doubleScaleAuto(int scaleSide, String startingPosition) {
		double angle1 = 15;
		
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if ((scaleSide == 1 && startingPosition == left) || (scaleSide == -1 && startingPosition == right)) { //Same side
			//Go forward to same side scale, turn around, pick up new cube, turn around, drop it off
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward part way
				if (DriveTrain.moveByDistance(130, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Move at angle and lift elevator
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(100, angle1 * -scaleSide, velocityMedium) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 3: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn Around
				if (DriveTrain.turnDegrees(scaleSide * (190 + angle1)) & Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);	
				}
				break;
			case 5: //Outtake
				Intake.open();
				if (DriveTrain.moveByDistance(50, velocitySlow)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 6:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(-50, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.turnDegrees(-scaleSide * (190 + angle1))) {
					state = nextState(state);
				}
				break;
			case 9:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 12:
				endAuto();
			}
		} else if ((scaleSide == 1 && startingPosition == right) || (scaleSide == -1 && startingPosition == left)) { //Opposite side
			//Cross field to go to opposite scale, drop it off, turn around, pick up new cube, turn around, drop it off
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.moveByDistance(190, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.turnDegrees(scaleSide * 90)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(194, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(-scaleSide * 90) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.moveByDistance(50, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.turnDegrees(scaleSide * 190) & Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);
				}
				break;
			case 8:
				Intake.open();
				if (DriveTrain.moveByDistance(45, velocitySlow)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 9:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.moveByDistance(-45, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (DriveTrain.turnDegrees(-scaleSide * 190)) {
					state = nextState(state);
				}
				break;
			case 12:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 13:
				if (DriveTrain.moveByDistance(-20, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 14:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 15:
				endAuto();
				break;
			}
		}

	}

	public void scaleSwitchAuto(int scaleSide, int switchSide, String startingPosition) {
		if (!isLifting && !Elevator.isAtFloor()) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		double angle1 = 16;
		
		if (scaleSide == switchSide) {
			//Same side both scale and switch
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
				break;
			case 1: //Move forward part way
				if (DriveTrain.moveByDistance(140, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Move at angle and lift elevator
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(80, angle1 * -scaleSide, velocityMedium) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 3: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 4: //Turn Around
				if (DriveTrain.turnDegrees(scaleSide * (185 + angle1)) & Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);	
				}
				break;
			case 5: //Outtake
				Intake.open();
				if (DriveTrain.moveByDistance(50, velocitySlow)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 6:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(8, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 9:
				if (Intake.outtakeForTime(timeToRollOut, previousTime))  {
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 12:
				endAuto();
				break;
			}
		} else if ((scaleSide == 1 && switchSide == -1 && startingPosition == left) || (scaleSide == -1 && switchSide == 1 && startingPosition == right)) {
			//Scale is on same side, switch is on opposite
			//So drop off cube at scale, go to switch and pick up cube and drop it off
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
			case 1: //Move forward part way
				if (DriveTrain.moveByDistance(130, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2: //Move at angle and lift elevator
				//Elevator.setHeight(scaleHeight);
				if (DriveTrain.moveByDistance(100, angle1 * -scaleSide, velocityMedium) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 3: //Outtake
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(scaleSide * (180 + angle1)) & Elevator.moveToFloorAuto(previousElevatorHeight)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.moveByDistance(35, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (DriveTrain.turnDegrees(scaleSide * 90)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.moveByDistance(150, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.turnDegrees(-scaleSide * 90)) {
					state = nextState(state);
				}
				break;
			case 9:
				Intake.open();
				if (DriveTrain.moveByDistance(15.65, velocitySlow)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 10:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (DriveTrain.moveByDistance(9, velocitySlow) & Elevator.moveToHeightAuto(switchSide, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 12:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 13:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 14:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 15:
				endAuto();
			}
		} else if ((scaleSide == -1 && switchSide == 1 && startingPosition == left) || (scaleSide == 1 && switchSide == -1 && startingPosition == right)) {
			//Switch is on same side, but scale is on opposite
			//So drop cube off in switch, pick up another cube, then go to scale
			switch (state) {
			case 0:
				setupAuto();
				state = nextState(state);
			case 1:
				if (DriveTrain.moveByDistance(190, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 2:
				if (DriveTrain.turnDegrees(-switchSide * 90)) {
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.moveByDistance(30, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 4:
				if (DriveTrain.turnDegrees(-switchSide * 90)) {
					state = nextState(state);
				}
				break;
			case 5:
				if (DriveTrain.moveByDistance(15, velocitySlow) & Elevator.moveToHeightAuto(switchHeight, switchHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 6:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 67:
				Intake.open();
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					Intake.close();
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.intakeForTime(timeToRollIn, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 9:
				if (DriveTrain.turnDegrees(switchSide * 90)) {
					state = nextState(state);
				}
				break;
			case 10:
				if (DriveTrain.moveByDistance(150, velocityFast)) {
					state = nextState(state);
				}
				break;
			case 11:
				if (DriveTrain.turnDegrees(switchSide * 90) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					state = nextState(state);
				}
				break;
			case 12:
				if (DriveTrain.moveByDistance(50, velocityMedium)) {
					state = nextState(state);
				}
				break;
			case 13:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 14:
				if (DriveTrain.moveByDistance(-15, velocitySlow)) {
					state = nextState(state);
				}
				break;
			case 15:
				if (Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 16:
				endAuto();
			}
		}
		
		//************   COMMENTED OUT CODE IS BROKEN MOTION PROFILING   **************//
		
	}
	
	//************   SCALE AUTO LEFT MOTION PROFILING   **************//

	
	MotionProfile profileScaleLeft_Same = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileScaleLeft_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileGetCubeFromScaleRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileGetCubeFromScaleLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileCubeToScaleRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileCubeToScaleLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	
	public void scaleAutoMPLeft(int scaleSide) {
		if (!Elevator.isAtFloor() && !isLifting) {
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
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft)) {
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
				if (DriveTrain.moveByDistance(-50, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.turnDegrees(-180, 10)) {
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
			
		} else if (scaleSide == -1) {//opposite side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Opposite);
				DriveTrain.startMotionProfile(profileScaleLeft_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Opposite) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 200)) {
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
				if (DriveTrain.turnDegrees(-180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight)) {
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
				if (DriveTrain.moveByDistance(-50, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.turnDegrees(180, 10)) {
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
			
		}
	}
	
	//************   SCALE AUTO RIGHT MOTION PROFILING   **************//
	
	MotionProfile profileScaleRight_Same = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileScaleRight_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	public void scaleAutoMPRight(int scaleSide) {
		if (!Elevator.isAtFloor() && !isLifting) {
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
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(-180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight)) {
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
				if (DriveTrain.moveByDistance(-50, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.turnDegrees(180, 10)) {
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
			
		} else if (scaleSide == 1) {//opposite side
			
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleRight_Opposite);
				DriveTrain.startMotionProfile(profileScaleRight_Opposite);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleRight_Opposite) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 200)) {
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
				if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft)) {
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
				if (DriveTrain.moveByDistance(-50, velocitySlow) & Elevator.moveToHeightAuto(scaleHeight, scaleHeight - previousElevatorHeight, 1)) {
					DriveTrain.startMotionProfile(profileCubeToScaleLeft);
					state = nextState(state);
				}
				break;
			case 7:
				if (DriveTrain.turnDegrees(-180, 10)) {
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
			
		}
	}
	
	//************   SWITCH AUTO MOTION PROFILING   **************//
	
	MotionProfile profileSwitchLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileSwitchRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	MotionProfile profileResetFromSwitchLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_R, leftSwitch_Middle_L, leftSwitch_Middle_L.length, true);
	MotionProfile profileResetFromSwitchRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_R, leftSwitch_Middle_L, leftSwitch_Middle_L.length, true);

	
	//SWITCH MOTION PROFILE AUTO 
	public void switchAutoMP(String startingPosition, int switchSide) {
		if (!Elevator.isAtFloor() && !isLifting) {
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
				if (DriveTrain.runMotionProfile(profileSwitchLeft) & Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileResetFromSwitchLeft);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					DriveTrain.startMotionProfile(profileResetFromSwitchLeft);
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.runMotionProfile(profileResetFromSwitchLeft) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 4:
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
				if (DriveTrain.runMotionProfile(profileSwitchRight) & Elevator.moveToHeightAuto(switchHeight, switchHeight - floorHeight, 1)) {
					DriveTrain.setupMotionProfile(profileResetFromSwitchRight);
					state = nextState(state);
				}
				break;
			case 2:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					DriveTrain.startMotionProfile(profileResetFromSwitchRight);
					state = nextState(state);
				}
				break;
			case 3:
				if (DriveTrain.runMotionProfile(profileResetFromSwitchRight) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 4:
				endAuto();
				break;
			}
		} else {
			SmartDashboard.putNumber("Auto State", -1);
			System.out.println("DON'T DO SWITCH AUTO ON THE SIDES STUPID");
		}
	}
	
	//************   SCALE-SWITCH AUTO LEFT MOTION PROFILING   **************//

	MotionProfile profileGetCubeFromScaleLeft_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);

	MotionProfile profileSwitchLeft_Side = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileBackAwayFromSwitchCube_Left = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length, true);
	
	MotionProfile profileCrossFieldToScaleLeftToRight = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	
	public void scaleSwitchAutoLeftMP(int scaleSide, int switchSide) {
		if (!Elevator.isAtFloor() && !isLifting) {
			Elevator.maintainHeight(previousElevatorHeight);
		}
		
		if (scaleSide == 1 && switchSide == 1) {
			switch (state) {
			case 0:
				setupAuto();
				DriveTrain.setupMotionProfile(profileScaleLeft_Same);
				DriveTrain.startMotionProfile(profileScaleLeft_Same);
				state = nextState(state);
				break;
			case 1:
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft)) {
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
				if (DriveTrain.moveByDistance(10, 10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileScaleLeft_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft_Opposite);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft_Opposite)) {
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
				if (DriveTrain.moveByDistance(10, -10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileScaleLeft_Opposite) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 200)) {
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
				if (DriveTrain.turnDegrees(-180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight)) {
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
				if (DriveTrain.moveByDistance(10, -10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileSwitchLeft_Side) & Elevator.moveToHeightAfterDriving(switchHeight, switchHeight - floorHeight, 1, 75)) {
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
				if (DriveTrain.runMotionProfile(profileCrossFieldToScaleLeftToRight) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - previousElevatorHeight, 1, 75)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-20, velocitySlow) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		}
	}
	
	//************   SCALE-SWITCH AUTO RIGHT MOTION PROFILING   **************//
	
	
	MotionProfile profileGetCubeFromScaleRight_Opposite = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	MotionProfile profileSwitchRight_Side = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	MotionProfile profileBackAwayFromSwitchCube_Right = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length, true);
	
	MotionProfile profileCrossFieldToScaleRightToLeft = new MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R, leftSwitch_Middle_L.length);
	
	public void scaleSwitchAutoRightMP(int scaleSide, int switchSide) {
		if (!Elevator.isAtFloor() && !isLifting) {
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
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(-180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight)) {
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
				if (DriveTrain.moveByDistance(10, -10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileScaleRight_Same) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 117)) {
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
				if (DriveTrain.turnDegrees(-180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleRight_Opposite);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRight_Opposite)) {
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
				if (DriveTrain.moveByDistance(10, 10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileScaleRight_Opposite) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - floorHeight, 1, 200)) {
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
				if (DriveTrain.turnDegrees(180) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft);
					state = nextState(state);
				}
				break;
			case 4:
				Intake.open();
				if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft)) {
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
				if (DriveTrain.moveByDistance(10, 10, velocitySlow, 1)) {
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
				if (DriveTrain.runMotionProfile(profileSwitchRight_Side) & Elevator.moveToHeightAfterDriving(switchHeight, switchHeight - floorHeight, 1, 50)) {
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
				if (DriveTrain.runMotionProfile(profileCrossFieldToScaleRightToLeft) & Elevator.moveToHeightAfterDriving(scaleHeight, scaleHeight - previousElevatorHeight, 1, 100)) {
					state = nextState(state);
				}
				break;
			case 7:
				if (Intake.outtakeForTime(timeToRollOut, previousTime)) {
					state = nextState(state);
				}
				break;
			case 8:
				if (DriveTrain.moveByDistance(-20, velocitySlow) & Elevator.moveToFloorAuto(previousElevatorHeight - floorHeight)) {
					state = nextState(state);
				}
				break;
			case 9:
				endAuto();
				break;
			}
		}
		
	}
}