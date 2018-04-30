package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain {

    static final WPI_TalonSRX LEFT_TALON = new WPI_TalonSRX(LEFT_TALON_CHANNEL);
    static final WPI_TalonSRX RIGHT_TALON = new WPI_TalonSRX(RIGHT_TALON_CHANNEL);
    static final WPI_VictorSPX LEFT_VICTOR = new WPI_VictorSPX(LEFT_VICTOR_CHANNEL);
    static final WPI_VictorSPX RIGHT_VICTOR = new WPI_VictorSPX(RIGHT_VICTOR_CHANNEL);

    static final ADXRS450_Gyro GYRO = new ADXRS450_Gyro();

    private static int turningStableTicks = 0;
    private static double previousTurningError = 0;
    private static double turningIntegral = 0;
    private static int endMoveTicks = 0;
    private static int timeOutTicks = 0;

    public static void setup() {
        LEFT_VICTOR.follow(LEFT_TALON);
        RIGHT_VICTOR.follow(RIGHT_TALON);

        // leftTalon.setSafetyEnabled(true);
        // rightTalon.setSafetyEnabled(true);

        LEFT_TALON.setNeutralMode(NeutralMode.Brake);
        RIGHT_TALON.setNeutralMode(NeutralMode.Brake);
        LEFT_VICTOR.setNeutralMode(NeutralMode.Brake);
        RIGHT_VICTOR.setNeutralMode(NeutralMode.Brake);

        LEFT_TALON.setInverted(true);
        RIGHT_TALON.setInverted(false);
        LEFT_VICTOR.setInverted(true);
        RIGHT_VICTOR.setInverted(false);

        LEFT_TALON.configOpenloopRamp(RAMP_RATE_TELEOP, 0);
        RIGHT_TALON.configOpenloopRamp(RAMP_RATE_TELEOP, 0);
        LEFT_TALON.configClosedloopRamp(RAMP_RATE_AUTO, 0);
        RIGHT_TALON.configClosedloopRamp(RAMP_RATE_AUTO, 0);

        LEFT_TALON.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        RIGHT_TALON.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

        LEFT_TALON.setSensorPhase(true);
        RIGHT_TALON.setSensorPhase(true);

        configPID();

        RIGHT_TALON.configMotionProfileTrajectoryPeriod(10, 0);
        LEFT_TALON.configMotionProfileTrajectoryPeriod(10, 0);
        RIGHT_TALON.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
        LEFT_TALON.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

        GYRO.reset();
        resetEncoders();
    }

    public static void configPID() {
        // Velocity PID in slot 0
        LEFT_TALON.config_kP(0, KP_L, 0);
        LEFT_TALON.config_kI(0, KI, 0);
        LEFT_TALON.config_kD(0, KD, 0);
        LEFT_TALON.config_kF(0, KF_L, 0);
        RIGHT_TALON.config_kP(0, KP_R, 0);
        RIGHT_TALON.config_kI(0, KI, 0);
        RIGHT_TALON.config_kD(0, KD, 0);
        RIGHT_TALON.config_kF(0, KF_R, 0);

        // Motion profile PID in slot 1
        LEFT_TALON.config_kP(1, KP_MP, 0);
        LEFT_TALON.config_kI(1, KI_MP, 0);
        LEFT_TALON.config_kD(1, KD_MP, 0);
        LEFT_TALON.config_kF(1, KF_MP, 0);
        RIGHT_TALON.config_kP(1, KP_MP, 0);
        RIGHT_TALON.config_kI(1, KI_MP, 0);
        RIGHT_TALON.config_kD(1, KD_MP, 0);
        RIGHT_TALON.config_kF(1, KF_MP, 0);
    }

    public static void reverseDriveTrain() {
        System.out.print("REVERSED DRIVE TRAIN");
        LEFT_TALON.setInverted(false);
        RIGHT_TALON.setInverted(true);
        LEFT_VICTOR.setInverted(false);
        RIGHT_VICTOR.setInverted(true);
    }

    public static void undoReverseDriveTrain() {
        System.out.print("RESET REVERSED DRIVE TRAIN");
        LEFT_TALON.setInverted(true);
        RIGHT_TALON.setInverted(false);
        LEFT_VICTOR.setInverted(true);
        RIGHT_VICTOR.setInverted(false);
    }

    public static void switchToVelocityMode() {
        LEFT_TALON.selectProfileSlot(0, 0);
        RIGHT_TALON.selectProfileSlot(0, 0);
    }

    public static void switchToMotionProfile() {
        LEFT_TALON.selectProfileSlot(1, 0);
        RIGHT_TALON.selectProfileSlot(1, 0);
    }

    // Tank drive for teleop control
    public static void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed = applyDeadband(leftSpeed);
        rightSpeed = applyDeadband(rightSpeed);

        double leftTarget = Math.copySign(leftSpeed * leftSpeed * leftSpeed * -1, leftSpeed)
                * MAX_TICKS_PER_100MS;
        double rightTarget = Math.copySign(rightSpeed * rightSpeed * rightSpeed * -1, rightSpeed)
                * MAX_TICKS_PER_100MS;

        // System.out.print("Speed Left:" + getFPS(leftTarget));
        // System.out.println("Speed right:" + getFPS(rightTarget));

        setVelocity(leftTarget, rightTarget);

        // Percent Output Mode
        // setSpeed(leftSpeed * leftSpeed * leftSpeed, rightSpeed * rightSpeed *
        // rightSpeed);

    }

    public static double applyDeadband(double speed) {
        if ((speed < DEADBAND && speed > 0) || (speed > -DEADBAND && speed < 0)) {
            speed = 0;
        } else if (speed > 0.92) {
            speed = 1;
        } else if (speed < -0.92) {
            speed = -1;
        }
        return speed;
    }

    // Autonomous move method
    public static boolean moveByDistance(double inches, double degrees, 
            double velocity, double timeOutSeconds) {
        double targetEncoderTicks = Math.abs(convertInchesToTicks(inches));
        double ticksSoFar = Math
                .abs((LEFT_TALON.getSelectedSensorPosition(0) 
                        + RIGHT_TALON.getSelectedSensorPosition(0)) / 2);
        double maxVelocity = convertVelocity(velocity);

        if (ticksSoFar >= targetEncoderTicks) {
            System.out.println("DONE WITH MOVE BY DISTANCE");
            endMoveTicks++;
            DriveTrain.stop();

            if (endMoveTicks > 5) { // Pause very briefly after the move
                // resetEncoders();
                // gyro.reset();
                Robot.AUTO.setPreviousFinalTurningError(0);
                endMoveTicks = 0;
                return true;
            } else {
                return false;
            }
        } else if (LEFT_TALON.getSelectedSensorVelocity(0) < 10 
                && RIGHT_TALON.getSelectedSensorVelocity(0) < 10
                && timeOutSeconds > 0) {
            timeOutTicks++;
            if (timeOutTicks > timeOutSeconds * 50) {
                DriveTrain.stop();
                Robot.AUTO.setPreviousFinalTurningError(0);
                endMoveTicks = 0;
                timeOutTicks = 0;
                return true;
            }
        } else {
            timeOutTicks = 0;
        }

        int direction = (inches > 0) ? 1 : -1;

        double ticksRemaining = targetEncoderTicks - ticksSoFar;
        // System.out.println("Ticks Remaining: " + ticksRemaining);
        double fractionRemaining = ticksRemaining / targetEncoderTicks;
        double scaledFraction = fractionRemaining * 3; // Start slowing down 2/3 of the way there
        if (scaledFraction > 1) {
            scaledFraction = 1;
        }

        double correctedHeading = degrees + Robot.AUTO.previousFinalTurningError();

        double degreeErrorLeft = (getGyro() - correctedHeading > 0) 
                ? getGyro() - correctedHeading : 0;
        double degreeErrorRight = (getGyro() - correctedHeading < 0) 
                ? correctedHeading - getGyro() : 0;

        double velocityLeft = maxVelocity + (KP_STRAIGHT_DRIVING * degreeErrorLeft * direction);
        double velocityRight = maxVelocity + (KP_STRAIGHT_DRIVING * degreeErrorRight * direction);

        double scaledSpeedR = scaledFraction * velocityRight;
        double scaledSpeedL = scaledFraction * velocityLeft;

        if (scaledSpeedR < MINIMUM_SPEED + (KP_STRAIGHT_DRIVING * degreeErrorRight * direction)) {
            scaledSpeedR = MINIMUM_SPEED + (KP_STRAIGHT_DRIVING * degreeErrorRight * direction);
        }
        if (scaledSpeedL < MINIMUM_SPEED + (KP_STRAIGHT_DRIVING * degreeErrorLeft * direction)) {
            scaledSpeedL = MINIMUM_SPEED + (KP_STRAIGHT_DRIVING * degreeErrorLeft * direction);
        }

        setVelocity(direction * -scaledSpeedL, direction * -scaledSpeedR);
        return false;
    }

    public static boolean moveByDistance(double inches, double heading, double velocity) {
        return moveByDistance(inches, heading, velocity, 0);
    }

    public static boolean moveByDistance(double inches, double velocity) {
        return moveByDistance(inches, 0, velocity, 0);
    }

    // Autonomous turn method
    public static boolean turnDegrees(double degrees, double timeOutDegreeTolerance) {
        // Positive degrees -> counterclockwise; negative degrees -> clockwise

        double currentAngle = getGyro();
        double error = degrees - currentAngle;
        double absoluteError = Math.abs(error);
        boolean inTolerance = false;

        int turnMultiplier = (error < 0) ? -1 : 1;
        double kP = (Math.abs(degrees) < 35) ? kPTurningSmall() : kPTurning();

        if (previousTurningError == 0) {
            previousTurningError = degrees;
        }

        System.out.println("Turning with current gyro angle " + getGyro() 
                + " and target " + degrees + " and error "
                + error + " and previous error " + previousTurningError);

        if (absoluteError <= TURNING_TOLERANCE) {
            DriveTrain.stop();
            inTolerance = true;

            // System.out.println("Has been stable within target's tolerance for " +
            // turningStableTicks + " iterations");
            System.out.println("Within tolerance with velocities L: " 
                    + LEFT_TALON.getSelectedSensorVelocity(0)
                    + " and R: " + RIGHT_TALON.getSelectedSensorPosition(0));

            if (LEFT_TALON.getSelectedSensorVelocity(0) < 75 
                    && RIGHT_TALON.getSelectedSensorVelocity(0) < 75) {
                System.out.println("STOP TURNING AT ANGLE: " + getGyro() 
                    + " with absolute error " + absoluteError);

                Robot.AUTO.setPreviousFinalTurningError(error);

                DriveTrain.stop();
                // gyro.reset();

                turningStableTicks = 0;
                previousTurningError = 0;
                turningIntegral = 0;

                return true;
            }
        } else if (LEFT_TALON.getSelectedSensorVelocity(0) < 25 
                && RIGHT_TALON.getSelectedSensorVelocity(0) < 25
                && absoluteError < timeOutDegreeTolerance && timeOutDegreeTolerance != 0) {
            turningStableTicks++;
            if (turningStableTicks > 100) {
                turningStableTicks = 0;
                previousTurningError = 0;
                turningIntegral = 0;

                Robot.AUTO.setPreviousFinalTurningError(error);

                DriveTrain.stop();
                return true;
            }
        } else {
            turningStableTicks = 0;
        }

        if (absoluteError < 1) {
            turningIntegral = 0; // Reset the integral to 0 if we are overshooting
        }

        double bias = minimumTurningOutput() * turnMultiplier;
        double proportional = error * kP;
        double derivative = (error - previousTurningError) * kDTurning();

        previousTurningError = error; // Reset previous error to current error

        double output = proportional + derivative + bias;
        if (absoluteError < I_ZONE) {
            turningIntegral += error;
            output += turningIntegral * kITurning();
        } else {
            turningIntegral = 0;
        }

        System.out.println(
                "Proportional: " + proportional + " integral: " + turningIntegral 
                + " derivative: " + derivative);

        if (Elevator.getCurrentPosition() > 50) {
            output *= (1 - (Elevator.getCurrentPosition() / (3.5 * SCALE_HEIGHT))); 
            // Reduce output with elevator at high heights
        }

        if (Math.abs(output) > MAX_TURNING_OUTPUT) {
            output = MAX_TURNING_OUTPUT * turnMultiplier;
        } /*
           * else if (Math.abs(output) < minimumTurningOutput) { output =
           * minimumTurningOutput * turnMultiplier; }
           */

        if (!inTolerance) {
            System.out.println("Setting turning speed: " + output 
                    + " with direction " + turnMultiplier);
            setSpeed(output, -1 * output);
        } else {
            System.out.println("Within tolerance, but velocity is too high. Stopping motors");
            stop();
        }
        return false;
    }

    public static boolean turnDegrees(double degrees) {
        return turnDegrees(degrees, 0);
    }

    // ***************************************//

    public static void setupMotionProfile(MotionProfile profile) {
        profile.setup();
        profile.startFilling();
    }

    public static void startMotionProfile(MotionProfile profile) {
        profile.periodic();
        profile.startMotionProfile();
    }

    public static boolean runMotionProfile(MotionProfile profile) {
        SetValueMotionProfile setValue = profile.getSetValue();
        LEFT_TALON.set(ControlMode.MotionProfile, setValue.value);
        RIGHT_TALON.set(ControlMode.MotionProfile, setValue.value);
        profile.periodic();

        if (profile.isMotionProfileComplete()) {
            return true;
        }
        return false;
    }

    // ***************************************//

    public static void setVelocity(double left, double right) {
        double elevatorHeight = Elevator.ENCODER.get();
        double scale = 1;
        if (Elevator.getCurrentPosition() > 40) {
            scale = 1 - (Elevator.getCurrentPosition() / (MAX_HEIGHT * 2.7));
        }

        LEFT_TALON.set(ControlMode.Velocity, left * scale);
        RIGHT_TALON.set(ControlMode.Velocity, right * scale);

        // System.out.println("Setting velocities L: " + left + " R: " + right);
        // System.out.println("Actual speed L: " +
        // leftTalon.getSelectedSensorVelocity(0) + " R: " +
        // rightTalon.getSelectedSensorVelocity(0));
    }

    public static void setSpeed(double left, double right) {
        LEFT_TALON.set(ControlMode.PercentOutput, left);
        RIGHT_TALON.set(ControlMode.PercentOutput, right);
    }

    public static void stop() {
        LEFT_TALON.set(ControlMode.PercentOutput, 0);
        RIGHT_TALON.set(ControlMode.PercentOutput, 0);
        // leftVictor.set(ControlMode.PercentOutput, 0);
        // rightVictor.set(ControlMode.PercentOutput, 0);
    }

    public static boolean wait(double time, double previousTime) {
        if (Robot.TIMER.get() - previousTime >= time) {
            return true;
        }
        return false;
    }

    public static double getGyro() {
        return -GYRO.getAngle();
    }

    public static void resetEncoders() {
        LEFT_TALON.setSelectedSensorPosition(0, 0, 0);
        RIGHT_TALON.setSelectedSensorPosition(0, 0, 0);
    }

}
