/*----------------------------------------------------------------------------*/
/* FRC Team 6705 2018 Robot Code: Power Up: Constants.java                    */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.Preferences;

//import edu.wpi.first.wpilibj.Preferences;

public class Constants {

    // Motor Controllers
    public static final int LEFT_TALON_CHANNEL = 0;
    public static final int RIGHT_TALON_CHANNEL = 1; 
    public static final int LEFT_VICTOR_CHANNEL = 0;
    public static final int RIGHT_VICTOR_CHANNEL = 1;
    public static final int LEFT_INTAKE_SPARK_CHANNEL = 2;
    public static final int RIGHT_INTAKE_SPARK_CHANNEL = 4; 
    public static final int ELEVATOR_SPARK_1 = 0;
    public static final int ELEVATOR_SPARK_2 = 1;

    // Solenoids
    public static final int INTAKE_SOLENOID_A = 0;
    public static final int INTAKE_SOLENOID_B = 1;
    public static final int LEFT_RAMP_SOLENOID_A = 4;
    public static final int LEFT_RAMP_SOLENOID_B = 5;
    public static final int INTAKE_ACTUATOR_SOLENOID_A = 3;
    public static final int INTAKE_ACTUATOR_SOLENOID_B = 2; 
    public static final int RIGHT_RAMP_SOLENOID_A = 8;
    public static final int RIGHT_RAMP_SOLENOID_B = 9;

    // Joysticks
    public static final int DRIVE_STICK_CHANNEL = 0;
    public static final int DPAD_CHANNEL = 0;
    public static final int LIFT_STICK_CHANNEL = 1;

    // Auto Starting Positions
    public static final String LEFT = "left";
    public static final String MIDDLE = "middle";
    public static final String RIGHT = "right";

    // Sensors
    public static final int ELEVATOR_ENCODER_SOURCE_A = 0; 
    public static final int ELEVATOR_ENCODER_SOURCE_B = 1;
    public static final int ELEVATOR_LIMIT_SWITCH_DIO = 2;

    // Rollers
    public static final double TIME_TO_ROLL_IN = 0.25; // seconds
    public static final double TIME_TO_ROLL_OUT = 0.5; // seconds
    public static final double MAX_ROLLERS_SPEED = 0.7;
    public static final double ROLLERS_SPEED_AUTO = 0.7; // [-1, 1]

    // Elevator Constants
    public static final double ELEVATOR_EQUILIBRIUM_SPEED_WITH_CUBE = 0.31; 
    public static final double ELEVATOR_EQUILIBRIUM_SPEED_NO_CUBE = 0.29;
    public static final double ELEVATOR_MAX_SPEED_UP = 1;
    public static final double ELEVATOR_MAX_SPEED_DOWN = -0.4; 
    public static final double ELEVATOR_MIN_SPEED_UP = 0.55;
    public static final double ELEVATOR_MIN_SPEED_DOWN = 0.1;
    public static final double TICKS_PER_ROTATION_ELEVATOR = 2048; 
    public static final double PULLEY_DIAMETER = 2.75;
    public static final double VERTICAL_INCHES_PER_TICK = 
            2 * (PULLEY_DIAMETER * Math.PI) / TICKS_PER_ROTATION_ELEVATOR; 
    public static final double ELEVATOR_TOLERANCE = 2; // Inches
    public static final double MAX_HEIGHT = 70;
    public static final double FLOOR_HEIGHT = 0.0;
    public static final double AUTO_DRIVE_HEIGHT = 5;
    public static final double SWITCH_HEIGHT = 30; 
    public static final double SCALE_HEIGHT = 61;
    public static final double ELEVATOR_RAMP_TIME = 0.2; // seconds

    // Drive Train
    public static final double WHEEL_RADIUS = 3.0; // inches
    public static final double TICKS_PER_REVOLUTION = 256 * 4; 
    // encoder ticks, multiplied by 4 because quadrature encoders do 4 pulses per count
    public static final double DISTANCE_PER_TICK = (WHEEL_RADIUS 
            * 2.0 * Math.PI) / TICKS_PER_REVOLUTION; //inches per tick
    public static final double DEADBAND = 0.08; // -1 to 1
    public static final double RAMP_RATE_AUTO = 1; // Seconds to ramp from 0 to full
    public static final double RAMP_RATE_TELEOP = 0.25;
    public static final double MAX_TICKS_PER_100MS = 870; 
    // This is the max speed in native units per 100 ms of the motors (percent output 100%)
    public static final double MAX_ERROR = 400;
    public static final double MINIMUM_SPEED = 75; // ticks per 100 ms
    public static final double KP_STRAIGHT_DRIVING = 45;

    // Turning PID
    private static double kPTurning = 0.013;
    private static double kPTurningSmall = 0.005; 
    private static double kDTurning = 0.028;
    private static double kITurning = 0;
    public static final double I_ZONE = 8; // Degree                                          
    public static final double TURNING_TOLERANCE = 4; // Degrees
    public static final double STEADY_TURNING_ITERATIONS = 5; //Iterations to exit turning PID loops
    public static final double MAX_TURNING_OUTPUT = 0.9;
    private static double minimumTurningOutput = 0.37;
    private static double minimumTurningOutput90 = 0.27;
    
    // Driving Speeds in Feet Per Second (FPS)
    public static final double VELOCITY_MAX = getFPS(MAX_TICKS_PER_100MS);
    public static final double VELOCITY_FAST = 10;
    public static final double VELOCITY_MEDIUM = 7;
    public static final double VELOCITY_MEDIUM_SLOW = 5.5;
    public static final double VELOCITY_SLOW = 5.5; 
    public static final double VELOCITY_TURNING_LEFT = 6;
    public static final double VELOCITY_TURNING_RIGHT = 6.3;
    public static final double VELOCITY_VERY_SLOW = 1.5;
    public static final double VELOCITY_QUITE_SLOW = 3.5;

    // PID for DriveTrain
    public static final double KP_L = (1023 * 0.05) / 225; // -(1023 * 0.1)/maxError
    public static final double KP_R = (1023 * 0.05) / 225;
    public static final double KD = 0; // kP * 10,//1023.0/maxError,
    public static final double KI = 0; // 1.023/maxError,
    public static final double KF_R = 1.164;
    public static final double KF_L = 1.164;

    // PID for Motion profile
    public static final double KP_MP = 0;
    public static final double KD_MP = 0;
    public static final double KI_MP = 0;
    public static final double KF_MP = 1.164;

    public static void getPreferences() {
        Preferences prefs = Preferences.getInstance();
        kPTurning = prefs.getDouble("kP_Turning", kPTurning);
        kITurning = prefs.getDouble("kI_Turning", kITurning);
        kDTurning = prefs.getDouble("kD_Turning", kDTurning);
        kPTurningSmall = prefs.getDouble("kP_Turning_Small", kPTurningSmall);
        minimumTurningOutput = prefs.getDouble("minimumTurningOutput", minimumTurningOutput);
        minimumTurningOutput90 = prefs.getDouble("minimumTurningOutput90", minimumTurningOutput90);
    }

    public static double convertInchesToTicks(double inches) {
        return (inches / (2 * Math.PI * WHEEL_RADIUS)) * TICKS_PER_REVOLUTION;
    }

    public static double convertTicksToInches(int ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * 2 * Math.PI * WHEEL_RADIUS;
    }

    public static double convertTicksToFeet(int ticks) {
        return convertTicksToInches(ticks) / 12;
    }

    public static double convertVelocity(double fps) {
        double rpm = (60 * 12 * fps) / (WHEEL_RADIUS * 2 * Math.PI);
        return (rpm * TICKS_PER_REVOLUTION) / 600.0; //Rev/Min * Ticks/Rev * Min/100ms->Ticks/100ms
    }

    public static double getFPS(double ticksPer100ms) {
        return (ticksPer100ms * 2 * WHEEL_RADIUS * Math.PI * 600)
                / (60 * 12 * TICKS_PER_REVOLUTION);
    }
    public static double kPTurning() {
        return kPTurning;
    }
    public static double kPTurningSmall() {
        return kPTurningSmall;
    }
    public static double kDTurning() {
        return kDTurning;
    }
    public static double kITurning() {
        return kITurning;
    }
    public static double minimumTurningOutput() {
        return minimumTurningOutput;
    }
}
