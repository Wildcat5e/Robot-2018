package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;
import static org.usfirst.frc.team6705.robot.MotionProfileDataSets.*;

//import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
    private int state = 0;
    private double previousTime = 0;
    private double previousElevatorHeight = FLOOR_HEIGHT;
    private double previousFinalTurningError = 0;
    private boolean setIsLifting = false;
    
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
        previousFinalTurningError = 0;
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
        previousTime = 0;
        previousElevatorHeight = FLOOR_HEIGHT;
        previousFinalTurningError = 0;
        setIsLifting = false;
        Elevator.setCompletedLift(false);
    }

    private int nextState(int current) {
        DriveTrain.resetEncoders();
        DriveTrain.GYRO.reset();
        previousTime = Robot.TIMER.get();
        previousElevatorHeight = Elevator.getCurrentPosition();
        SmartDashboard.putNumber("Auto State", current + 1);
        DriveTrain.stop();
        Elevator.setCompletedLift(false);
        // DriveTrain.leftTalon.clearMotionProfileTrajectories();
        // DriveTrain.rightTalon.clearMotionProfileTrajectories();
        return current + 1;

    }

    // ***************************************************************************//

    // MotionProfile exampleProfile = new MotionProfile(DriveTrain.leftTalon,
    // DriveTrain.rightTalon, leftSwitch_Middle_L, leftSwitch_Middle_R,
    // leftSwitch_Middle_L.length);

    public void testAuto() {
        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        System.out.println("Test Auto");
        switch (state) {
        case 0:
            setupAuto();
            state = nextState(state);
            break;
        /*
         * case 1: if (DriveTrain.moveByDistance(50, velocitySlow) &
         * Elevator.moveToHeightAuto(switchHeight, switchHeight, 1)) { state =
         * nextState(state); } break;
         */
        case 1:
            Intake.actuateUp();
            state = nextState(state);
            break;
        case 2:
            endAuto();
            break;
        default:
        }
    }

    // ***************************************************************************//

    private MotionProfile profileStraight = new MotionProfile(DriveTrain.LEFT_TALON, 
            DriveTrain.RIGHT_TALON,
            LEFT_SWITCH_MIDDLE_START_L, LEFT_SWITCH_MIDDLE_START_R, 
            LEFT_SWITCH_MIDDLE_START_L.length);

    public void motionProfileTest() {

        if (!setIsLifting && !Elevator.isAtFloor()) {
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
                // DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
                state = nextState(state);
            }
            break;
        /*
         * case 3: if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeft)) { state
         * = nextState(state); } break;
         */
        case 2:
            endAuto();
            break;
        default:
        }

    }

    // ***************************************************************************//

    public void baselineAuto(int scaleSide, String startingPosition) {
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        switch (state) {
        case 0:
            setupAuto();
            state = nextState(state);
            break;
        case 1:
            if (DriveTrain.moveByDistance(120, VELOCITY_MEDIUM)) {
                state = nextState(state);
            }
            break;
        case 2:
            endAuto();
            break;
        default:
        }
    }

    // ***************************************************************************//

    public void switchAuto1Cube(int switchSide, int scaleSide, String startingPosition) {
        // SwitchSide: 1 -> left, -1 -> right
        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }
        double diagonalDistance = (switchSide == 1) ? 54 : 47; /* 58 : 50; */

        if ((startingPosition == LEFT && switchSide == 1) 
                || (startingPosition == RIGHT && switchSide == -1)) { // Same side switch auto
            return;
        } else if (startingPosition == MIDDLE) {
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(5,
                        VELOCITY_SLOW)) { 
                    /* && Elevator.moveToHeightAuto(autoDriveHeight, autoDriveHeight, 1) */
                    state = nextState(state);
                }
                break;
            case 2: // Turn
                if (DriveTrain.turnDegrees(45 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 3: // Move across field in correct direction
                if (DriveTrain.moveByDistance(diagonalDistance, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(-45 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 5: // Move elevator
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 6: // Move forward rest of distance
                if (DriveTrain.moveByDistance(29, 0, VELOCITY_SLOW, 2)) {
                    state = nextState(state);
                }
                break;
            case 7: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(-29, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.turnDegrees(switchSide * 45)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.moveByDistance(-diagonalDistance, VELOCITY_MEDIUM)) {
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
            default:
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
        }
    }

    // ***************************************************************************//

    public void switchAuto2Cube(int switchSide, int scaleSide, String startingPosition) {
        // SwitchSide: 1 -> left, -1 -> right
        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }
        double diagonalDistance = (switchSide == 1) ? 54 : 47; /* 58 : 50; */

        if ((startingPosition == LEFT && switchSide == 1) 
                || (startingPosition == RIGHT && switchSide == -1)) { // Same side switch auto
            return;
        } else if (startingPosition == MIDDLE) {
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(5, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 2: // Turn
                if (DriveTrain.turnDegrees(45 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 3: // Move across field in correct direction
                if (DriveTrain.moveByDistance(diagonalDistance, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(-45 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 5: // Move forward rest of distance
                if (DriveTrain.moveByDistance(29, 0, VELOCITY_SLOW, 2)) {
                    state = nextState(state);
                }
                break;
            case 6: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(-29, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.turnDegrees(switchSide * 60)) {
                    state = nextState(state);
                }
                break;
            case 9:
                Intake.open();
                if (DriveTrain.moveByDistance(-40, VELOCITY_MEDIUM)
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.turnDegrees(switchSide * -60)) {
                    state = nextState(state);
                }
                break;
            case 11:
                Intake.open();
                if (DriveTrain.moveByDistance(20, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 12:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 13:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 14:
                if (DriveTrain.turnDegrees(60 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 15:
                if (DriveTrain.moveByDistance(40, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 16:
                if (DriveTrain.turnDegrees(-60 * switchSide)) {
                    state = nextState(state);
                }
                break;
            case 17: // Move forward rest of distance
                if (DriveTrain.moveByDistance(29, 0, VELOCITY_SLOW, 2)) {
                    state = nextState(state);
                }
                break;
            case 18: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 19:
                if (DriveTrain.moveByDistance(-29, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 20:
                endAuto();
                break;
            default:
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
        }
    }

    // ***************************************************************************//

    public void switchAuto3Cube(int switchSide, int scaleSide, String startingPosition) {
        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }
        // Parameters Left Right
        double initialDistance = (switchSide == 1) ? 78 : 68;
        double initialAngle = (switchSide == 1) ? 30 : 20;
        double backAndForthDistance = (switchSide == 1) ? 74 : 64;
        double grabCubeDistance = (switchSide == 1) ? 32 : 32;
        double grabSecondCubeDistance = (switchSide == 1) ? 50 : 50;
        double angle2 = (switchSide == 1) ? 30 : 20;
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

        if (startingPosition == MIDDLE) {
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(5, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 2:
                if (DriveTrain.turnDegrees(switchSide * initialAngle, 6)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.moveByDistance(initialDistance, VELOCITY_MEDIUM_SLOW)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.moveByDistance(-backAndForthDistance, VELOCITY_MEDIUM_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.turnDegrees(-switchSide * angle2, 6)) {
                    state = nextState(state);
                }
                break;
            case 7:
                Intake.open();
                if (DriveTrain.moveByDistance(grabCubeDistance, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-grabCubeDistance, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.turnDegrees(switchSide * angle2, 6)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.moveByDistance(backAndForthDistance, VELOCITY_MEDIUM_SLOW)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 13:
                if (DriveTrain.moveByDistance(-backAndForthDistance, VELOCITY_MEDIUM_SLOW)
                        & Elevator.moveToHeightAuto(11, previousElevatorHeight - 20, -1)) {
                    state = nextState(state);
                }
                break;
            case 14:
                if (DriveTrain.turnDegrees(-switchSide * angle2, 6)) {
                    state = nextState(state);
                }
                break;
            case 15:
                Intake.open();
                if (DriveTrain.moveByDistance(grabSecondCubeDistance, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 16:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 17:
                if (DriveTrain.moveByDistance(-grabCubeDistance, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 18:
                endAuto();
                break;
            default:
            /*
             * case 18: if (DriveTrain.turnDegrees(finalAngle)) { state = nextState(state);
             * } break; case 19: if (DriveTrain.moveByDistance(finalDistance, velocityFast))
             * { state = nextState(state); } break; case 20: if
             * (DriveTrain.turnDegrees(-scaleSide * 70)) { state = nextState(state); }
             * break; case 21: endAuto(); break;
             */
            }

        }
    }

    // ***************************************************************************//

    public void singleScaleAuto(int scaleSide, String startingPosition) {

        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }
        System.out.println("Scale side " + scaleSide + "Starting pos " + startingPosition);
        if ((scaleSide == 1 && startingPosition == LEFT) 
                || (scaleSide == -1 && startingPosition == RIGHT)) { // Same side
            // Go forward and turn to same side scale
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1: // Move forward part way
                if (DriveTrain.moveByDistance(280, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2:
                Intake.actuateUp();
                if (Elevator.moveToHeightAuto(SCALE_HEIGHT, SCALE_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(90 * -scaleSide, 15)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.moveByDistance(10, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 5: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-15, VELOCITY_SLOW)) {
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
            default:
            }
        } else if ((scaleSide == 1 && startingPosition == RIGHT) 
                || (scaleSide == -1 && startingPosition == LEFT)) { // Opposite side
            // Cross field to drop off cube at opposite side scale
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(191, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2:
                if (DriveTrain.turnDegrees(scaleSide * 90, 6)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.moveByDistance(184, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(-scaleSide * 90, 8)) {
                    state = nextState(state);
                }
                break;
            case 5:
                if (Elevator.moveToHeightAuto(SCALE_HEIGHT,
                        SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(30, VELOCITY_MEDIUM)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(-23, VELOCITY_SLOW)) {
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
            default:
            }
        }
    }

    // ***************************************************************************//

    public void doubleScaleAuto(int scaleSide, String startingPosition) {
        // double angle1 = 20;

        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if ((scaleSide == 1 && startingPosition == LEFT) 
                || (scaleSide == -1 && startingPosition == RIGHT)) { // Same side
            // Go forward to same side scale, turn around, pick up new cube, turn around,
            // drop it off
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1: // Move forward part way
                if (DriveTrain.moveByDistance(120, -5 * scaleSide, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2: // Move at angle and lift elevator
                // Elevator.setHeight(scaleHeight);
                if (DriveTrain.moveByDistance(106, -5 * scaleSide, VELOCITY_FAST)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 3: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 4: // Turn Around
                if (DriveTrain.turnDegrees(scaleSide * -160)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 5: // Outtake
                Intake.open();
                if (DriveTrain.moveByDistance(55, VELOCITY_MEDIUM)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 6:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(-55, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.turnDegrees(scaleSide * 180)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(10, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 13:
                endAuto();
                break;
            default:
            }
        } else if ((scaleSide == 1 && startingPosition == RIGHT) 
                || (scaleSide == -1 && startingPosition == LEFT)) { // Opposite side
            // Cross field to go to opposite scale, drop it off, turn around, pick up new
            // cube, turn around, drop it off
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(185, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2:
                if (DriveTrain.turnDegrees(scaleSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.moveByDistance(184, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(-scaleSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.moveByDistance(45, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.turnDegrees(-scaleSide * 170) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 8:
                Intake.open();
                if (DriveTrain.moveByDistance(45, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 9:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.moveByDistance(-45, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, SCALE_HEIGHT, 1)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.turnDegrees(scaleSide * 170)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 13:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 14:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 15:
                endAuto();
                break;
            default:
            }
        }

    }

    public void scaleSwitchAuto(int scaleSide, int switchSide, String startingPosition) {
        if (!setIsLifting && !Elevator.isAtFloor()) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        double angle1 = 16;

        if (scaleSide == switchSide) {
            // Same side both scale and switch
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1: // Move forward part way
                if (DriveTrain.moveByDistance(140, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2: // Move at angle and lift elevator
                // Elevator.setHeight(scaleHeight);
                if (DriveTrain.moveByDistance(80, angle1 * -scaleSide, VELOCITY_MEDIUM)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 3: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 4: // Turn Around
                if (DriveTrain.turnDegrees(scaleSide * (185 + angle1))
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 5: // Outtake
                Intake.open();
                if (DriveTrain.moveByDistance(50, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 6:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(8, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.moveByDistance(-15, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 12:
                endAuto();
                break;
            default:
            }
        } else if ((scaleSide == 1 && switchSide == -1 && startingPosition == LEFT)
                || (scaleSide == -1 && switchSide == 1 && startingPosition == RIGHT)) {
            // Scale is on same side, switch is on opposite
            // So drop off cube at scale, go to switch and pick up cube and drop it off
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1: // Move forward part way
                if (DriveTrain.moveByDistance(130, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2: // Move at angle and lift elevator
                // Elevator.setHeight(scaleHeight);
                if (DriveTrain.moveByDistance(100, angle1 * -scaleSide, VELOCITY_MEDIUM)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 3: // Outtake
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(scaleSide * (180 + angle1))
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.moveByDistance(35, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.turnDegrees(scaleSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(150, VELOCITY_FAST)) {
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
                if (DriveTrain.moveByDistance(15.65, VELOCITY_SLOW)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 10:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.moveByDistance(9, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(switchSide, 
                                SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 13:
                if (DriveTrain.moveByDistance(-15, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 14:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 15:
                endAuto();
                break;
            default:
            }
        } else if ((scaleSide == -1 && switchSide == 1 && startingPosition == LEFT)
                || (scaleSide == 1 && switchSide == -1 && startingPosition == RIGHT)) {
            // Switch is on same side, but scale is on opposite
            // So drop cube off in switch, pick up another cube, then go to scale
            switch (state) {
            case 0:
                setupAuto();
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.moveByDistance(190, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 2:
                if (DriveTrain.turnDegrees(-switchSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.moveByDistance(30, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 4:
                if (DriveTrain.turnDegrees(-switchSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.moveByDistance(15, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 67:
                Intake.open();
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 7:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(-15, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.turnDegrees(switchSide * 90)) {
                    state = nextState(state);
                }
                break;
            case 10:
                if (DriveTrain.moveByDistance(150, VELOCITY_FAST)) {
                    state = nextState(state);
                }
                break;
            case 11:
                if (DriveTrain.turnDegrees(switchSide * 90)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (DriveTrain.moveByDistance(50, VELOCITY_MEDIUM)) {
                    state = nextState(state);
                }
                break;
            case 13:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 14:
                if (DriveTrain.moveByDistance(-15, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 15:
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 16:
                endAuto();
                break;
            default:
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
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if (scaleSide == 1) { // same side

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-50, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.turnDegrees(-180, 10)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == -1) { // opposite side

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleRightStart);
                DriveTrain.startMotionProfile(profileLeftScaleRightStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 200)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(-180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-50, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT,
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.turnDegrees(180, 10)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
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
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if (scaleSide == -1) { // same side

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(-180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-50, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.turnDegrees(180, 10)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == 1) { // opposite side

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleLeftStart);
                DriveTrain.startMotionProfile(profileRightScaleLeftStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT,
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 200)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-50, VELOCITY_SLOW)
                        & Elevator.moveToHeightAuto(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.turnDegrees(-180, 10)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
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
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if (startingPosition == MIDDLE && switchSide == 1) { // Left switch auto
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftSwitchMiddleStart);
                DriveTrain.startMotionProfile(profileLeftSwitchMiddleStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                SWITCH_HEIGHT - FLOOR_HEIGHT, 1)) {
                    DriveTrain.setupMotionProfile(profileResetFromSwitchLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    DriveTrain.startMotionProfile(profileResetFromSwitchLeft);
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.runMotionProfile(profileResetFromSwitchLeft)
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    DriveTrain.setupMotionProfile(profileLeftSwitchMiddleStart);
                    state = nextState(state);
                }
                break;
            case 4:
                Intake.open();
                if (DriveTrain.moveByDistance(55, VELOCITY_MEDIUM)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 5:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-55, VELOCITY_MEDIUM)) {
                    DriveTrain.startMotionProfile(profileLeftSwitchMiddleStart);
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.runMotionProfile(profileLeftSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    DriveTrain.setupMotionProfile(profileResetFromSwitchLeft);
                    DriveTrain.startMotionProfile(profileResetFromSwitchLeft);
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.runMotionProfile(profileResetFromSwitchLeft)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                Intake.open();
                if (DriveTrain.moveByDistance(55, VELOCITY_MEDIUM)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 11:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (DriveTrain.moveByDistance(-30, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 13:
                endAuto();
                break;
            default:
            }
        } else if (startingPosition == MIDDLE && switchSide == -1) { // Right switch auto
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightSwitchMiddleStart);
                DriveTrain.startMotionProfile(profileRightSwitchMiddleStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                                SWITCH_HEIGHT - FLOOR_HEIGHT, 1)) {
                    DriveTrain.setupMotionProfile(profileResetFromSwitchRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    DriveTrain.startMotionProfile(profileResetFromSwitchRight);
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.runMotionProfile(profileResetFromSwitchRight)
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    DriveTrain.setupMotionProfile(profileRightSwitchMiddleStart);
                    state = nextState(state);
                }
                break;
            case 4:
                Intake.open();
                if (DriveTrain.moveByDistance(55, VELOCITY_MEDIUM)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 5:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.moveByDistance(-55, VELOCITY_MEDIUM)) {
                    DriveTrain.startMotionProfile(profileRightSwitchMiddleStart);
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.runMotionProfile(profileRightSwitchMiddleStart)
                        & Elevator.moveToHeightAuto(SWITCH_HEIGHT, SWITCH_HEIGHT, 1)) {
                    DriveTrain.setupMotionProfile(profileResetFromSwitchRight);
                    DriveTrain.startMotionProfile(profileResetFromSwitchRight);
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.runMotionProfile(profileResetFromSwitchRight)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                Intake.open();
                if (DriveTrain.moveByDistance(55, VELOCITY_MEDIUM)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 11:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 12:
                if (DriveTrain.moveByDistance(-30, VELOCITY_SLOW)) {
                    state = nextState(state);
                }
                break;
            case 13:
                endAuto();
                break;
            default:
            }
        } else {
            SmartDashboard.putNumber("Auto State", -1);
            System.out.println("DON'T DO SWITCH AUTO ON THE SIDES STUPID");
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
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if (scaleSide == 1 && switchSide == 1) {
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, 10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }
        } else if (scaleSide == 1 && switchSide == -1) { // Same side scale, opposite side switch

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleLeftStart);
                DriveTrain.startMotionProfile(profileLeftScaleLeftStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT,
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeftOpposite);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    DriveTrain.startMotionProfile(profileGetCubeFromScaleLeftOpposite);
                    state = nextState(state);
                }
                break;
            case 4:
                Intake.open();
                if (DriveTrain.runMotionProfile(profileGetCubeFromScaleLeftOpposite)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 5:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, -10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == -1 && switchSide == -1) { // Opposite side both scale and switch

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileLeftScaleRightStart);
                DriveTrain.startMotionProfile(profileLeftScaleRightStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileLeftScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 200)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(-180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT, 
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, -10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == -1 && switchSide == 1) { // Opposite side scale, same side switch
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileSwitchLeftSide);
                DriveTrain.startMotionProfile(profileSwitchLeftSide);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileSwitchLeftSide)
                        & Elevator.moveToHeightAfterDriving(SWITCH_HEIGHT,
                                SWITCH_HEIGHT - FLOOR_HEIGHT, 1, 75)) {
                    DriveTrain.setupMotionProfile(profileBackAwayFromSwitchCubeLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                Intake.open();
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 4:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    DriveTrain.startMotionProfile(profileBackAwayFromSwitchCubeLeft);
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.runMotionProfile(profileBackAwayFromSwitchCubeLeft)) {
                    DriveTrain.setupMotionProfile(profileCrossFieldToScaleLeftToRight);
                    DriveTrain.startMotionProfile(profileCrossFieldToScaleLeftToRight);
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.runMotionProfile(profileCrossFieldToScaleLeftToRight)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1, 75)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 9:
                endAuto();
                break;
            default:
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
        if (!Elevator.isAtFloor() && !setIsLifting) {
            Elevator.maintainHeight(previousElevatorHeight);
        }

        if (scaleSide == -1 && switchSide == -1) { // Same side both scale and switch
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(-180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, -10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }
        } else if (scaleSide == -1 && switchSide == 1) { // Same side scale, opposite side switch

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleRightStart);
                DriveTrain.startMotionProfile(profileRightScaleRightStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightScaleRightStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 117)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleRightOpposite);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(-180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    DriveTrain.startMotionProfile(profileGetCubeFromScaleRightOpposite);
                    state = nextState(state);
                }
                break;
            case 4:
                Intake.open();
                if (DriveTrain.runMotionProfile(profileGetCubeFromScaleRightOpposite)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 5:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, 10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == 1 && switchSide == 1) { // Opposite side both scale and switch

            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileRightScaleLeftStart);
                DriveTrain.startMotionProfile(profileRightScaleLeftStart);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileRightScaleLeftStart)
                        & Elevator.moveToHeightAfterDriving(SCALE_HEIGHT,
                                SCALE_HEIGHT - FLOOR_HEIGHT, 1, 200)) {
                    DriveTrain.setupMotionProfile(profileGetCubeFromScaleLeft);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                if (DriveTrain.turnDegrees(180) 
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
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
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 6:
                if (Elevator.moveToHeightAuto(SWITCH_HEIGHT,
                        SWITCH_HEIGHT - previousElevatorHeight, 1)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (DriveTrain.moveByDistance(10, 10, VELOCITY_SLOW, 1)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 9:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight)) {
                    state = nextState(state);
                }
                break;
            case 10:
                endAuto();
                break;
            default:
            }

        } else if (scaleSide == 1 && switchSide == -1) { // Opposite side scale, same side switch
            switch (state) {
            case 0:
                setupAuto();
                DriveTrain.setupMotionProfile(profileSwitchRightSide);
                DriveTrain.startMotionProfile(profileSwitchRightSide);
                state = nextState(state);
                break;
            case 1:
                if (DriveTrain.runMotionProfile(profileSwitchRightSide)
                        & Elevator.moveToHeightAfterDriving(SWITCH_HEIGHT,
                                SWITCH_HEIGHT - FLOOR_HEIGHT, 1, 50)) {
                    DriveTrain.setupMotionProfile(profileBackAwayFromSwitchCubeRight);
                    state = nextState(state);
                }
                break;
            case 2:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 3:
                Intake.open();
                if (Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    Intake.close();
                    state = nextState(state);
                }
                break;
            case 4:
                if (Intake.intakeForTime(TIME_TO_ROLL_IN, previousTime)) {
                    DriveTrain.startMotionProfile(profileBackAwayFromSwitchCubeRight);
                    state = nextState(state);
                }
                break;
            case 5:
                if (DriveTrain.runMotionProfile(profileBackAwayFromSwitchCubeRight)) {
                    DriveTrain.setupMotionProfile(profileCrossFieldToScaleRightToLeft);
                    DriveTrain.startMotionProfile(profileCrossFieldToScaleRightToLeft);
                    state = nextState(state);
                }
                break;
            case 6:
                if (DriveTrain.runMotionProfile(profileCrossFieldToScaleRightToLeft) & Elevator
                        .moveToHeightAfterDriving(SCALE_HEIGHT, 
                                SCALE_HEIGHT - previousElevatorHeight, 1, 100)) {
                    state = nextState(state);
                }
                break;
            case 7:
                if (Intake.outtakeForTime(TIME_TO_ROLL_OUT, previousTime)) {
                    state = nextState(state);
                }
                break;
            case 8:
                if (DriveTrain.moveByDistance(-20, VELOCITY_SLOW)
                        & Elevator.moveToFloorAuto(previousElevatorHeight - FLOOR_HEIGHT)) {
                    state = nextState(state);
                }
                break;
            case 9:
                endAuto();
                break;
            default:
            }
        }

    }
    public void setPreviousElevatorHeight(double value) {
        previousElevatorHeight = value;
    }
    public void setPreviousFinalTurningError(double value) {
        previousFinalTurningError = value;
    }
    public double previousFinalTurningError() {
        return previousFinalTurningError;
    }
    public boolean isLifting() {
        return setIsLifting;
    }
    public void setIsLifting(boolean status) {
        setIsLifting = status;
    }
}