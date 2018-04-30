package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.*;


public class Elevator /* extends PIDSubsystem */ {

    static final Spark SPARK_1 = new Spark(ELEVATOR_SPARK_1);
    static final Spark SPARK_2 = new Spark(ELEVATOR_SPARK_2);

    static final SpeedControllerGroup MOTOR = new SpeedControllerGroup(SPARK_1, SPARK_2);

    static final Encoder ENCODER = new Encoder(ELEVATOR_ENCODER_SOURCE_A, 
            ELEVATOR_ENCODER_SOURCE_B, false, EncodingType.k4X);

    static final DigitalInput LIMIT_SWITCH = new DigitalInput(ELEVATOR_LIMIT_SWITCH_DIO);

    private static boolean hasCompletedLift = false;

    // static PIDController pid = new PIDController(kP_Lift, kI_Lift, kD_Lift,
    // kF_Lift, encoder, motor);

    // static PIDMotor motor1;
    // static PIDMotor motor2;

    public static void setup() {
        // encoder = new Encoder(elevatorEncoderSourceA, elevatorEncoderSourceB, false,
        // EncodingType.k4X);
        ENCODER.reset();
        ENCODER.setDistancePerPulse(VERTICAL_INCHES_PER_TICK);

    }

    public static double convertTicksToVerticalInches(double ticks) {
        return ticks * VERTICAL_INCHES_PER_TICK;
    }

    public static double getCurrentPosition() {
        return convertTicksToVerticalInches(ENCODER.get()) + FLOOR_HEIGHT;
    }

    public static void set(double speed) { 
        // Takes in a value from -1 to 1, scales it to the values that work for the
        // elevator, and sets the motor to that speed
        if ((speed >= 0 && getCurrentPosition() < MAX_HEIGHT) || (speed < 0 && !isAtFloor())) {
            System.out.println("Setting lift speed (unscaled): " + speed);
            if (speed < 0) {
                double downSpeed = (speed * (ELEVATOR_MIN_SPEED_DOWN - ELEVATOR_MAX_SPEED_DOWN))
                        + ELEVATOR_MIN_SPEED_DOWN;
                System.out.println("Setting elevator speed down: " + downSpeed);
                MOTOR.set(downSpeed);
            } else if (speed >= 0) {
                double upSpeed = (speed * (1 - ELEVATOR_MIN_SPEED_UP)) + ELEVATOR_MIN_SPEED_UP;
                System.out.println("Setting elevator speed up: " + upSpeed);
                MOTOR.set(upSpeed);
            }
        } else {
            System.out.println("REACHED A LIMIT");
            // stop();
        }
    }

    public static void setTeleop(double speed, int intervalsCounted) {
        double intervals = (intervalsCounted > ELEVATOR_RAMP_TIME * 50) 
                ? ELEVATOR_RAMP_TIME * 50 : intervalsCounted;
        /*
         * if (speed < 0 && getCurrentPosition() < floorHeight +
         * elevatorDecelerationDistance) { Elevator.set(speed * ((getCurrentPosition() -
         * floorHeight)/elevatorDecelerationDistance)); } else if (speed > 0 &&
         * getCurrentPosition() > maximumHeight - elevatorDecelerationDistance) {
         * Elevator.set(speed * ((maximumHeight -
         * getCurrentPosition())/elevatorDecelerationDistance)); }
         */ 
        if (intervals < ELEVATOR_RAMP_TIME * 50 && speed > 0) {
            double actualSpeed = (intervals / (ELEVATOR_RAMP_TIME * 50)) 
                    * speed * ELEVATOR_MAX_SPEED_UP;
            System.out.println("Setting teleop speed " + actualSpeed);
            Elevator.set(actualSpeed);
        } else {
            set(speed);
        }
    }

    public static void maintainHeight(double height) {
        if (getCurrentPosition() < height + ELEVATOR_TOLERANCE) {
            System.out.println("Trying to maintain height " + height 
                    + "Current Position is " + getCurrentPosition());
            double speed = (Intake.SOLENOID.get() == DoubleSolenoid.Value.kReverse) 
                    ? ELEVATOR_EQUILIBRIUM_SPEED_WITH_CUBE
                    : ELEVATOR_EQUILIBRIUM_SPEED_NO_CUBE;
            // elevatorMinimumSpeedUp * (height - getCurrentPosition());
            // speed = (height - getCurrentPosition()) * 0.1;
            // System.out.println("Equilibrium speed: " + speed);
            MOTOR.set(speed);
        } /*
           * else { motor.set(0); }
           */
    }

    public static void stop() {
        SPARK_1.set(0);
        SPARK_2.set(0);
    }

    public static boolean isAtFloor() {
        return !LIMIT_SWITCH.get();
    }

    public static void moveToHeight(double targetHeight, double distanceToLift, int direction) {
        // int direction = (currentHeight > targetHeight) ? -1 : 1;

        double distanceRemaining = Math.abs(getCurrentPosition() - targetHeight);
        double fractionRemaining = Math.abs(distanceRemaining / distanceToLift);
        System.out.println("Fraction Remaining: " + fractionRemaining);
        System.out.println("Distance Remaining: " + distanceRemaining);

        if (fractionRemaining > 1) {
            fractionRemaining = 1;
        }

        double scaledFraction = fractionRemaining * 1.25;

        double fractionLifted = 1 - fractionRemaining;
        if (fractionLifted < 0.01) {
            fractionLifted = 0.01;
        }

        double scaledFractionLifted = fractionLifted * 5;

        if (fractionLifted < 0.2) {
            scaledFraction = scaledFractionLifted;
        } else if (scaledFraction > 1) {
            scaledFraction = 1;
        }

        Elevator.set(direction * scaledFraction);
    }

    public static boolean moveToHeightAuto(double targetHeight, 
            double totalDistanceToLift, int direction) {
        double currentHeight = getCurrentPosition();
        // int direction = (currentHeight > targetHeight) ? -1 : 1;

        double distanceRemaining = currentHeight - targetHeight;
        double absDistance = Math.abs(distanceRemaining);
        System.out.println("Move to height auto");

        if ((currentHeight < targetHeight + ELEVATOR_TOLERANCE 
                && currentHeight > targetHeight - ELEVATOR_TOLERANCE)
                || hasCompletedLift) {
            hasCompletedLift = true;
            Robot.AUTO.setIsLifting(false);
            Robot.AUTO.setPreviousElevatorHeight(getCurrentPosition());
            System.out.println("AUTO ELEVATOR MOVE DONE");
            return true;
        } else {
            Robot.AUTO.setIsLifting(true);
        }

        double fractionRemaining = Math.abs(absDistance / totalDistanceToLift);
        if (fractionRemaining > 1) {
            fractionRemaining = 1;
        }

        double scaledFraction = fractionRemaining * 1.25;

        double fractionLifted = 1 - fractionRemaining;
        if (fractionLifted < 0.01) {
            fractionLifted = 0.01;
        }

        double scaledFractionLifted = fractionLifted * 5;

        if (fractionLifted < 0.2) {
            scaledFraction = scaledFractionLifted;
        } else if (scaledFraction > 1) {
            scaledFraction = 1;
        }

        Elevator.set(direction * scaledFraction);

        return false;
    }

    public static boolean moveToFloorAuto(double totalDistanceToLift) {
        double currentHeight = getCurrentPosition();
        int direction = -1;

        System.out.println("Move to floor auto");

        double distanceRemaining = Math.abs(currentHeight - FLOOR_HEIGHT);
        if (Elevator.isAtFloor()) {
            ENCODER.reset();
            // Elevator.stop();
            Robot.AUTO.setIsLifting(false);
            return true;
        } else {
            Robot.AUTO.setIsLifting(true);
        }

        double fractionRemaining = Math.abs(distanceRemaining / totalDistanceToLift);
        if (fractionRemaining > 1) {
            fractionRemaining = 1;
        }

        double scaledFraction = fractionRemaining * 1.25;

        double fractionLifted = 1 - fractionRemaining;
        if (fractionLifted < 0.01) {
            fractionLifted = 0.01;
        }

        double scaledFractionLifted = fractionLifted * 5;

        if (fractionLifted < 0.2) {
            scaledFraction = scaledFractionLifted;
        } else if (scaledFraction > 1) {
            scaledFraction = 1;
        }

        Elevator.set(direction * scaledFraction);
        return false;

    }

    public static boolean moveToHeightAfterDriving(double targetHeight,
            double totalDistanceToLift, int direction,
            double distanceInches) {
        double leftDistance = DriveTrain.LEFT_TALON.getSelectedSensorPosition(0);
        double rightDistance = DriveTrain.RIGHT_TALON.getSelectedSensorPosition(0);
        double averageDistance = (leftDistance + rightDistance) / 2;

        if (averageDistance < convertInchesToTicks(distanceInches)) {
            System.out.println("Has only driven " + averageDistance 
                    + " ticks, so not yet lifting elevator");
            Robot.AUTO.setIsLifting(false);
            return false;
        } else {
            System.out.println(
                    "Has driven at least " + convertInchesToTicks(distanceInches) 
                    + " ticks, so now lifting elevator");
            return moveToHeightAuto(targetHeight, totalDistanceToLift, direction);
        }
    }
    static boolean hasCompletedLift() {
        return hasCompletedLift;
    }
    static void setCompletedLift(boolean status) {
        hasCompletedLift = status;
    }
    public static enum ElevatorState {
        MANUAL, FLOOR, SWITCH, SCALE, TEST;
    }

}