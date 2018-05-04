package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import org.usfirst.frc.team6705.robot.Autonomous.CurrentRobotState;

import edu.wpi.first.wpilibj.*;

public class Intake {

    static final Spark ROLLERS_LEFT = new Spark(LEFT_INTAKE_SPARK_CHANNEL);
    static final Spark ROLLERS_RIGHT = new Spark(RIGHT_INTAKE_SPARK_CHANNEL);

    static final DoubleSolenoid SOLENOID = new DoubleSolenoid(INTAKE_SOLENOID_A, INTAKE_SOLENOID_B);
    static final DoubleSolenoid ACTUATOR_1 = new DoubleSolenoid(INTAKE_ACTUATOR_SOLENOID_A, 
            INTAKE_ACTUATOR_SOLENOID_B);
    // static DoubleSolenoid actuator2 = new DoubleSolenoid(intakeActuatorSolenoidA,
    // intakeActuatorSolenoidB);

    public static void setup() {
        SOLENOID.set(DoubleSolenoid.Value.kReverse);
        ROLLERS_RIGHT.setInverted(false);
        ROLLERS_LEFT.setInverted(false);

    }

    public static void intake() {
        ROLLERS_LEFT.set(-ROLLERS_SPEED_AUTO);
        ROLLERS_RIGHT.set(ROLLERS_SPEED_AUTO);
    }

    public static void outtake() {
        ROLLERS_LEFT.set(ROLLERS_SPEED_AUTO);
        ROLLERS_RIGHT.set(-ROLLERS_SPEED_AUTO);
    }

    public static boolean actuateUp() {
        System.out.print("Actuate Intake Up");
        ACTUATOR_1.set(DoubleSolenoid.Value.kReverse);
        return true;
    }

    public static boolean actuateDown() {
        System.out.print("Actuate Intake Down");
        ACTUATOR_1.set(DoubleSolenoid.Value.kForward);
        return true;
    }

    public static void roll(double speed) {
        System.out.println("Rolling speed: " + speed);
        ROLLERS_LEFT.set(-speed * MAX_ROLLERS_SPEED);
        ROLLERS_RIGHT.set(speed * MAX_ROLLERS_SPEED);
    }

    
    public static boolean outtakeForTime(double time, CurrentRobotState robotState) {
        double startTime = robotState.getPreviousTime();
        if (Robot.TIMER.get() - startTime >= time) {
            stopRollers();
            return true;
        }
        outtake();
        return false;
    }

    
    public static boolean intakeForTime(double time, CurrentRobotState robotState) {
        double startTime = robotState.getPreviousTime();
        if (Robot.TIMER.get() - startTime >= time) {
            stopRollers();
            return true;
        }
        intake();
        return false;
    }

    public static void stopRollers() {

        ROLLERS_LEFT.set(0);
        ROLLERS_RIGHT.set(0);
    }

    public static boolean open() {
        System.out.print("Open Pneumatics");
        SOLENOID.set(DoubleSolenoid.Value.kForward);
        return true;
    }

    public static boolean close() {
        System.out.println("Close Pneumatics");
        SOLENOID.set(DoubleSolenoid.Value.kReverse);
        return true;
    }

    /*
     * public static void angleUp() { actuator1.set(DoubleSolenoid.Value.kForward);
     * actuator2.set(DoubleSolenoid.Value.kOff); }
     *
     * public static void angleDiagonal() {
     * actuator1.set(DoubleSolenoid.Value.kOff);
     * actuator2.set(DoubleSolenoid.Value.kOff); }
     *
     * public static void angleDown() { actuator1.set(DoubleSolenoid.Value.kOff);
     * actuator2.set(DoubleSolenoid.Value.kForward); }
     */

    public static enum IntakeState {
        MANUAL, INTAKING, OUTTAKING;
    }

}
