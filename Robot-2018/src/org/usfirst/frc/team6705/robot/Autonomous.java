package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;
import static org.usfirst.frc.team6705.robot.MotionProfileDataSets.*;

import java.util.*;
import java.util.function.*;

//import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    private int state = 0;
    private CurrentRobotState currentState = new CurrentRobotState();
    private List<Supplier<Boolean>> commandList = new ArrayList<>();
    /*
     * Note: Distance from alliance wall to front of switch = 140 in Distance from
     * front of robot to front of switch = 101 in Distance from middle of robot to
     * middle of switch = 148.5 Distance from middle of robot to middle of scale =
     * 304.5 Middle - line up touching edge to exchange zone Left & Right - line up
     * robot at corner diagonal of field
     *
     */
    
    private void setupAuto() {
        DriveTrain.resetEncoders();
        DriveTrain.GYRO.reset();
        currentState.setPreviousFinalTurningError(0);
        Elevator.setCompletedLift(false);
        // DriveTrain.leftTalon.clearMotionProfileTrajectories();
        // DriveTrain.rightTalon.clearMotionProfileTrajectories();
    }

    private void endAuto() {
        DriveTrain.stop();
        Intake.stopRollers();
        // Elevator.stop();
        DriveTrain.LEFT_TALON.set(ControlMode.PercentOutput, 0);
        DriveTrain.RIGHT_TALON.set(ControlMode.PercentOutput, 0);
        System.out.println("Finished auto routine!");
    }

    public void resetAuto() {
        state = 0;
        currentState.setPreviousTime(0);
        currentState.setPreviousElevatorHeight(FLOOR_HEIGHT);
        currentState.setPreviousFinalTurningError(0);
        currentState.setIsLifting(false);
        Elevator.setCompletedLift(false);
    }

    private int nextState(int current) {
        DriveTrain.resetEncoders();
        DriveTrain.GYRO.reset();
        currentState.setPreviousTime(Robot.TIMER.get());
        currentState.setPreviousElevatorHeight(Elevator.getCurrentPosition());
        SmartDashboard.putNumber("Auto State", current + 1);
        DriveTrain.stop();
        Elevator.setCompletedLift(false);
        // DriveTrain.leftTalon.clearMotionProfileTrajectories();
        // DriveTrain.rightTalon.clearMotionProfileTrajectories();
        return current + 1;

    }
    private void executeAuto() {
        if (state - 1 < commandList.size()) {
            if (commandList.get(state - 1).get()) {
                state = nextState(state);
            }
        } else {
            endAuto();
        }
    }
    // ***************************************************************************//

    // MotionProfile exampleProfile = new MotionProfile(DriveTrain.leftTalon,
    // DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R,
    // leftSwitch_Middle_L.length);

    public void testAuto() {
        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }

        System.out.println("Test Auto");
        if (state == 0) {
            setupAuto();
            commandList = new ArrayList<>();
            commandList.add(() -> Intake.actuateUp());
            state = nextState(state);
        } else {
            executeAuto();
        }
        
    }

    // ***************************************************************************//

    private MotionProfile profileStraight = new MotionProfile(DriveTrain.LEFT_TALON, 
            DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_L, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_L.length);

    public void motionProfileTest() {

        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }
        if (state == 0) {
            setupAuto();
            DriveTrain.setupMotionProfile(profileStraight);
            DriveTrain.startMotionProfile(profileStraight);
            commandList.add(() -> DriveTrain.runMotionProfile(profileStraight));
            state = nextState(state);
        } else {
            executeAuto();
        }

    }

    // ***************************************************************************//

    public void baselineAuto(int scaleSide, String startingPosition) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }
        if (state == 0) {
            setupAuto();
            commandList = new ArrayList<>();
            commandList.add(() -> DriveTrain.moveByDistance(120, VELOCITY_MEDIUM, currentState));
            state = nextState(state);
        } else {
            executeAuto();
        }
        
    }

    // ***************************************************************************//

    public void switchAuto1Cube(int switchSide, int scaleSide, String startingPosition) {
        // SwitchSide: 1 -> left, -1 -> right
        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }
        double diagonalDistance = (switchSide == 1) ? 54 : 47; /* 58 : 50; */

        if ((startingPosition == LEFT && switchSide == 1) 
                || (startingPosition == RIGHT && switchSide == -1)) { // Same side switch auto
            return;
        } else if (startingPosition == MIDDLE) {
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(5, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(45 * switchSide, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(diagonalDistance, 
                        VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-45 * switchSide, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(29, 0, 
                        VELOCITY_SLOW, 2, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-29, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * 45, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-diagonalDistance, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * -45, currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
        }
    }

    // ***************************************************************************//

    public void switchAuto2Cube(int switchSide, int scaleSide, String startingPosition) {
        // SwitchSide: 1 -> left, -1 -> right
        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }
        double diagonalDistance = (switchSide == 1) ? 54 : 47; /* 58 : 50; */

        if ((startingPosition == LEFT && switchSide == 1) 
                || (startingPosition == RIGHT && switchSide == -1)) { // Same side switch auto
            return;
        } else if (startingPosition == MIDDLE) {
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(5, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(45 * switchSide, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(diagonalDistance, 
                        VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(-45 * switchSide, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(29, 0, 
                        VELOCITY_SLOW, 2, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-29, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-40, VELOCITY_MEDIUM, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * -60, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(20, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(60 * switchSide, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(40, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(-60 * switchSide, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(29, 0, 
                        VELOCITY_SLOW, 2, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-29, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
        }
    }

    // ***************************************************************************//

    public void switchAuto3Cube(int switchSide, int scaleSide, String startingPosition) {
        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }
        // Parameters Left Right
        double initialDistance = (switchSide == 1) ? 78 : 68;
        double initialAngle = (switchSide == 1) ? 30 : 20;
        double backAndForthDistance = (switchSide == 1) ? 74 : 64;
        double grabCubeDistance = (switchSide == 1) ? 32 : 32;
        double grabSecondCubeDistance = (switchSide == 1) ? 50 : 50;
        double angle2 = (switchSide == 1) ? 30 : 20;
        /*
        double finalDistance = (switchSide == scaleSide) ? 80 : 100;
        double finalAngle = 0;

        if (scaleSide == 1 && switchSide == 1) {
            finalAngle = 90;
        } else if (scaleSide == 1 && switchSide == -1) {
            finalAngle = 70;
        } else if (scaleSide == -1 && switchSide == -1) {
            finalAngle = -90;
        } else if (scaleSide == -1 && switchSide == 1) {
            finalAngle = -70;
        }
        */
        if (startingPosition == MIDDLE) {
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(5, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * initialAngle, 
                        6, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(initialDistance, 
                        VELOCITY_MEDIUM_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-backAndForthDistance, 
                        VELOCITY_MEDIUM_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-switchSide * angle2, 
                        6, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(grabCubeDistance, 
                        VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-grabCubeDistance, 
                        VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * angle2,
                        6, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(backAndForthDistance, 
                        VELOCITY_MEDIUM_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-backAndForthDistance, 
                        VELOCITY_MEDIUM_SLOW, currentState)
                        & Elevator.moveToHeightAuto(11, currentState, -1));
                commandList.add(() -> DriveTrain.turnDegrees(-switchSide * angle2, 
                        6, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(grabSecondCubeDistance,
                        VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-grabCubeDistance,
                        VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
            /*
             * case 18: if (DriveTrain.turnDegrees(finalAngle)) { state = nextState(state);
             * } break; case 19: if (DriveTrain.moveByDistance(finalDistance, velocityFast))
             * { state = nextState(state); } break; case 20: if
             * (DriveTrain.turnDegrees(-scaleSide * 70)) { state = nextState(state); }
             * break; case 21: endAuto(); break;
             */
            

        }
    }

    // ***************************************************************************//

    public void singleScaleAuto(int scaleSide, String startingPosition) {

        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }
        System.out.println("Scale side " + scaleSide + "Starting pos " + startingPosition);
        if ((scaleSide == 1 && startingPosition == LEFT) 
                || (scaleSide == -1 && startingPosition == RIGHT)) { // Same side
            // Go forward and turn to same side scale
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(280, VELOCITY_FAST, currentState));
                commandList.add(() -> Intake.actuateUp());
                commandList.add(() -> Elevator.moveToHeightAuto(SCALE_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(90 * -scaleSide, 15, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(10, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-15, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if ((scaleSide == 1 && startingPosition == RIGHT) 
                || (scaleSide == -1 && startingPosition == LEFT)) { // Opposite side
            // Cross field to drop off cube at opposite side scale
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(191, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * 90, 6, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(184, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-scaleSide * 90, 8, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SCALE_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(30, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-23, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        }
    }

    // ***************************************************************************//

    public void doubleScaleAuto(int scaleSide, String startingPosition) {
        // double angle1 = 20;

        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }

        if ((scaleSide == 1 && startingPosition == LEFT) 
                || (scaleSide == -1 && startingPosition == RIGHT)) { // Same side
            // Go forward to same side scale, turn around, pick up new cube, turn around,
            // drop it off
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(120, -5 * scaleSide, 
                        VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(106, -5 * scaleSide, 
                        VELOCITY_FAST, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * -160, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-55, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * 180, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(10, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if ((scaleSide == 1 && startingPosition == RIGHT) 
                || (scaleSide == -1 && startingPosition == LEFT)) { // Opposite side
            // Cross field to go to opposite scale, drop it off, turn around, pick up new
            // cube, turn around, drop it off
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(185, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(184, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-scaleSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(45, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-scaleSide * 170, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(45, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-45, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * 170, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        }

    }

    public void scaleSwitchAuto(int scaleSide, int switchSide, String startingPosition) {
        if (!currentState.getIsLifting() && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(currentState);
        }

        double angle1 = 16;

        if (scaleSide == switchSide) {
            // Same side both scale and switch
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(140, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(80, 
                        angle1 * -scaleSide, VELOCITY_MEDIUM, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * (185 + angle1), 
                        currentState)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(50, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(8, VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-15, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if ((scaleSide == 1 && switchSide == -1 && startingPosition == LEFT)
                || (scaleSide == -1 && switchSide == 1 && startingPosition == RIGHT)) {
            // Scale is on same side, switch is on opposite
            // So drop off cube at scale, go to switch and pick up cube and drop it off
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(130, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(100, angle1 * -scaleSide, 
                        VELOCITY_MEDIUM, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * (180 + angle1), 
                        currentState)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.moveByDistance(35, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(scaleSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(150, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-scaleSide * 90, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(15.65, 
                        VELOCITY_SLOW, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(9, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(switchSide, 
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-15, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if ((scaleSide == -1 && switchSide == 1 && startingPosition == LEFT)
                || (scaleSide == 1 && switchSide == -1 && startingPosition == RIGHT)) {
            // Switch is on same side, but scale is on opposite
            // So drop cube off in switch, pick up another cube, then go to scale
            if (state == 0) {
                setupAuto();
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.moveByDistance(190, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-switchSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(30, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-switchSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(15, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-15, VELOCITY_SLOW, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * 90, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(150, VELOCITY_FAST, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(switchSide * 90, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(50,
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-15, VELOCITY_SLOW, currentState));
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        }

    }

    // ************ SCALE AUTO LEFT MOTION PROFILING **************//

    private MotionProfile profileLeftScaleLeftStart = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);
    private MotionProfile profileLeftScaleRightStart = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);

    private MotionProfile profileGetCubeFromScaleRight = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);
    private MotionProfile profileGetCubeFromScaleLeft = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);

    // MotionProfile profileCubeToScaleRight = new
    // MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon,
    // leftSwitchMiddleStart_R, leftSwitchMiddleStart_R,
    // leftSwitchMiddleStart_R.length);
    // MotionProfile profileCubeToScaleLeft = new
    // MotionProfile(DriveTrain.leftTalon, DriveTrain.rightTalon,
    // leftSwitchMiddleStart_R, leftSwitchMiddleStart_R,
    // leftSwitchMiddleStart_R.length);

    public void scaleAutoMPLeft(int scaleSide) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }

        if (scaleSide == 1) { // same side
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-50, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(-180, 10, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        } else if (scaleSide == -1) { // opposite side
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleRightStart);
                DriveTrain.startMotionProfile(profileLeftScaleRightStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 200));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-50, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(180, 10, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        }
    }

    // ************ SCALE AUTO RIGHT MOTION PROFILING **************//

    private MotionProfile profileRightScaleRightStart =
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);
    private MotionProfile profileRightScaleLeftStart = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);

    public void scaleAutoMPRight(int scaleSide) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }

        if (scaleSide == -1) { // same side
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-50, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(180, 10, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        } else if (scaleSide == 1) { // opposite side
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleLeftStart);
                DriveTrain.startMotionProfile(profileRightScaleLeftStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT,
                                currentState, 1, 200));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-50, VELOCITY_SLOW, currentState)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.turnDegrees(-180, 10, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        }
    }

    // ************ SWITCH AUTO MOTION PROFILING **************//

    private MotionProfile profileLeftSwitchMiddleStart = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_L, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_L.length);
    private MotionProfile profileRightSwitchMiddleStart =
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_L, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_R.length);

    private MotionProfile profileResetFromSwitchLeft = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_L, 
            LEFT_SWITCH_MIDDLE_START_R.length, true);
    private MotionProfile profileResetFromSwitchRight = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_R, LEFT_SWITCH_MIDDLE_START_L, 
            LEFT_SWITCH_MIDDLE_START_R.length, true);

    // SWITCH MOTION PROFILE AUTO
    public void switchAutoMP(String startingPosition, int switchSide) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }

        if (startingPosition == MIDDLE && switchSide == 1) { // Left switch auto
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftSwitchMiddleStart);
                DriveTrain.startMotionProfile(profileLeftSwitchMiddleStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileResetFromSwitchLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileResetFromSwitchLeft));
                commandList.add(() -> DriveTrain.runMotionProfile(profileResetFromSwitchLeft)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileLeftSwitchMiddleStart));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileLeftSwitchMiddleStart));
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileResetFromSwitchLeft));
                commandList.add(() -> DriveTrain.startMotionProfile(profileResetFromSwitchLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.runMotionProfile(profileResetFromSwitchLeft)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-30, VELOCITY_SLOW, currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if (startingPosition == MIDDLE && switchSide == -1) { // Right switch auto
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightSwitchMiddleStart);
                DriveTrain.startMotionProfile(profileRightSwitchMiddleStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                currentState, 1));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileResetFromSwitchRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileResetFromSwitchRight));
                commandList.add(() -> DriveTrain.runMotionProfile(profileResetFromSwitchRight)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileRightSwitchMiddleStart));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileRightSwitchMiddleStart));
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, currentState, 1));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileResetFromSwitchRight));
                commandList.add(() -> DriveTrain.startMotionProfile(profileResetFromSwitchRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.runMotionProfile(profileResetFromSwitchRight)
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.moveByDistance(55, 
                        VELOCITY_MEDIUM, currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-30, VELOCITY_SLOW, currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
            System.out.println("DON'T DO SWITCH AUTO ON THE SIDES you smart person :)");
        }
    }

    // ************ SCALE-SWITCH AUTO LEFT MOTION PROFILING **************//

    private MotionProfile profileGetCubeFromScaleLeftOpposite = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);

    private MotionProfile profileSwitchLeftSide = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);
    private MotionProfile profileBackAwayFromSwitchCubeLeft = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length, true);

    private MotionProfile profileCrossFieldToScaleLeftToRight = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);

    public void scaleSwitchAutoLeftMP(int scaleSide, int switchSide) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }

        if (scaleSide == 1 && switchSide == 1) {
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(10, 
                        10, VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if (scaleSide == 1 && switchSide == -1) { // Same side scale, opposite side switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileGetCubeFromScaleLeftOpposite));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileGetCubeFromScaleLeftOpposite));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileGetCubeFromScaleLeftOpposite));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        currentState, 1));
                commandList.add(() -> DriveTrain.
                        moveByDistance(10, -10, VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        } else if (scaleSide == -1 && switchSide == -1) { // Opposite side both scale and switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleRightStart);
                DriveTrain.startMotionProfile(profileLeftScaleRightStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileLeftScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 200));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(10, -10, 
                        VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if (scaleSide == -1 && switchSide == 1) { // Opposite side scale, same side switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileSwitchLeftSide);
                DriveTrain.startMotionProfile(profileSwitchLeftSide);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileSwitchLeftSide)
                        & Elevator.moveToHeightAfterDriving(SWITCH_HEIGHT,
                                currentState, 1, 75));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileBackAwayFromSwitchCubeLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileBackAwayFromSwitchCubeLeft));
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileBackAwayFromSwitchCubeLeft));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileCrossFieldToScaleLeftToRight));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileCrossFieldToScaleLeftToRight));
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileCrossFieldToScaleLeftToRight)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 75));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }
        }
    }

    // ************ SCALE-SWITCH AUTO RIGHT MOTION PROFILING **************//

    private MotionProfile profileGetCubeFromScaleRightOpposite = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);

    private MotionProfile profileSwitchRightSide = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);
    private MotionProfile profileBackAwayFromSwitchCubeRight = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length, true);

    private MotionProfile profileCrossFieldToScaleRightToLeft = 
            new MotionProfile(DriveTrain.LEFT_TALON, DriveTrain.RIGHT_TALON,
            STRAIGHT_TEST_L, STRAIGHT_TEST_L, STRAIGHT_TEST_L.length);

    public void scaleSwitchAutoRightMP(int scaleSide, int switchSide) {
        if (!Elevator.isAtFloor() && !currentState.getIsLifting()) {
            Elevator.maintainHeight(currentState);
        }

        if (scaleSide == -1 && switchSide == -1) { // Same side both scale and switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleRight));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(10, -10, 
                        VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                
                state = nextState(state);
            } else {
                executeAuto();
            }
        } else if (scaleSide == -1 && switchSide == 1) { // Same side scale, opposite side switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 117));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileGetCubeFromScaleRightOpposite));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(-180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileGetCubeFromScaleRightOpposite));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileGetCubeFromScaleRightOpposite));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        currentState, 1));
                commandList.add(() -> DriveTrain.
                        moveByDistance(10, 10, VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        } else if (scaleSide == 1 && switchSide == 1) { // Opposite side both scale and switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleLeftStart);
                DriveTrain.startMotionProfile(profileRightScaleLeftStart);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileRightScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT,
                                currentState, 1, 200));
                commandList.add(() -> DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.turnDegrees(180, currentState) 
                        & Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> DriveTrain.startMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.open());
                commandList.add(() -> DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        currentState, 1));
                commandList.add(() -> DriveTrain.moveByDistance(10, 10, 
                        VELOCITY_SLOW, 1, currentState));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                state = nextState(state);
            } else {
                executeAuto();
            }

        } else if (scaleSide == 1 && switchSide == -1) { // Opposite side scale, same side switch
            if (state == 0) {
                setupAuto();
                DriveTrain.setupMotionProfile(profileSwitchRightSide);
                DriveTrain.startMotionProfile(profileSwitchRightSide);
                commandList = new ArrayList<>();
                commandList.add(() -> DriveTrain.runMotionProfile(profileSwitchRightSide)
                        & Elevator.moveToHeightAfterDriving(SWITCH_HEIGHT,
                                currentState, 1, 50));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileBackAwayFromSwitchCubeRight));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> Intake.open());
                commandList.add(() -> Elevator.moveToFloorAuto(currentState));
                commandList.add(() -> Intake.close());
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileBackAwayFromSwitchCubeRight));
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileBackAwayFromSwitchCubeRight));
                commandList.add(() -> DriveTrain.
                        setupMotionProfile(profileCrossFieldToScaleRightToLeft));
                commandList.add(() -> DriveTrain.
                        startMotionProfile(profileCrossFieldToScaleRightToLeft));
                commandList.add(() -> DriveTrain.
                        runMotionProfile(profileCrossFieldToScaleRightToLeft) & Elevator
                        .moveToHeightAfterDriving(SCALE_HEIGHT, 
                                currentState, 1, 100));
                commandList.add(() -> Intake.outtakeForTime(TIME_TO_ROLL_OUT, currentState));
                commandList.add(() -> DriveTrain.moveByDistance(-20, VELOCITY_SLOW, currentState)
                        & Elevator.moveToFloorAuto(currentState));
                
                state = nextState(state);
            } else {
                executeAuto();
            }
        }

    }
    class CurrentRobotState {
        private double previousTime = 0;
        private double previousElevatorHeight = FLOOR_HEIGHT;
        private double previousFinalTurningError = 0;
        private boolean setIsLifting = false;
        public double getPreviousTime() {
            return previousTime;
        }
        public void setPreviousTime(double d) {
            previousTime = d;
        }
        public double getPreviousElevatorHeight() {
            return previousElevatorHeight;
        }
        public void setPreviousElevatorHeight(double d) {
            previousElevatorHeight = d;
        }
        public double getPreviousFinalTurningError() {
            return previousFinalTurningError;
        }
        public void setPreviousFinalTurningError(double d) {
            previousFinalTurningError = d;
        }
        public boolean getIsLifting() {
            return setIsLifting;
        }
        public void setIsLifting(boolean b) {
            setIsLifting = b;
        }
    }
}