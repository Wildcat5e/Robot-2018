package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Notifier;

public class MotionProfile {

    private MotionProfileStatus statusLeft = new MotionProfileStatus();
    private MotionProfileStatus statusRight = new MotionProfileStatus();
    private double pos = 0;
    private double vel = 0;
    private double heading = 0;
    private WPI_TalonSRX talonLeft;
    private WPI_TalonSRX talonRight;

    private int state = 0;
    private boolean start = false;
    private boolean finished = false;

    private double[][] profileLeft;
    private double[][] profileRight;
    private int numberOfPoints;
    private boolean reversed = false;

    private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;

    private static final int MINIMUM_TRAGECTORY_POINTS = 5;

    private int timeoutLoops = -1;
    private static final int TIMEOUT_LIMIT = 10;
    
    private Notifier notifier = new Notifier(new PeriodicRunnable());

    public MotionProfile(WPI_TalonSRX left, WPI_TalonSRX right, 
            double[][] arrayLeft, double[][] arrayRight,
            int totalCount) {
        talonLeft = left;
        talonRight = right;

        profileLeft = arrayLeft;
        profileRight = arrayRight;

        numberOfPoints = totalCount;
    }

    public MotionProfile(WPI_TalonSRX left, WPI_TalonSRX right, 
            double[][] arrayLeft, double[][] arrayRight,
            int totalCount, boolean isReverse) {
        talonLeft = left;
        talonRight = right;

        profileLeft = arrayLeft;
        profileRight = arrayRight;

        numberOfPoints = totalCount;

        reversed = isReverse;
    }

    public void setup() {
        talonLeft.changeMotionControlFramePeriod(5);
        talonRight.changeMotionControlFramePeriod(5);

        finished = false;

        notifier.startPeriodic(0.005);
    }

    public void reset() {
        talonLeft.clearMotionProfileTrajectories();
        talonRight.clearMotionProfileTrajectories();

        setValue = SetValueMotionProfile.Disable;
        state = 0;
        timeoutLoops = -1;
        start = false;
    }

    public void startFilling() {
        startFillingPoints(profileLeft, profileRight, numberOfPoints);
    }

    public void periodic() {

        talonLeft.getMotionProfileStatus(statusLeft);
        talonRight.getMotionProfileStatus(statusRight);

        if (timeoutLoops < 0) {
            System.out.println("Motion Profile Disabled");
        } else {
            if (timeoutLoops == 0) {
                System.out.println("Motion profile error, operation took too long");
            } else {
                timeoutLoops--;
            }
        }

        if (talonLeft.getControlMode() != ControlMode.MotionProfile) {
            state = 0;
            timeoutLoops = -1;
        } else {
            // In Motion Profile Mode

            switch (state) {
            case 0:
                // Not started yet
                if (start) {
                    start = false;
                    if (!reversed) {
                        DriveTrain.reverseDriveTrain();
                    }
                    setValue = SetValueMotionProfile.Disable;
                    System.out.println("Starting Motion Profile");
                    state = 1;
                    timeoutLoops = TIMEOUT_LIMIT;
                }
                break;
            case 1:
                // Wait for points to start filling
                System.out.println("Waiting for points to be filled");
                if (statusLeft.btmBufferCnt > MINIMUM_TRAGECTORY_POINTS
                        && statusRight.btmBufferCnt > MINIMUM_TRAGECTORY_POINTS) {
                    setValue = SetValueMotionProfile.Enable;
                    state = 2;
                    timeoutLoops = TIMEOUT_LIMIT;
                }
                break;
            case 2:
                if (!statusLeft.isUnderrun && !statusRight.isUnderrun) {
                    timeoutLoops = TIMEOUT_LIMIT;
                }

                System.out.println("Running MP, has not finished");

                if (statusLeft.activePointValid 
                        && statusLeft.isLast 
                        && statusRight.activePointValid
                        && statusRight.isLast) {
                    // Motion Profile complete, load next one
                    finished = true;
                    if (!reversed) {
                        DriveTrain.undoReverseDriveTrain();
                    }
                    System.out.println("FINISHED MOTION PROFILE");
                    setValue = SetValueMotionProfile.Hold;
                    state = 0;
                    timeoutLoops = -1;
                }
                break;
            default:        
            }

        }

    }

    private TrajectoryDuration getTrajectoryDuration(int durationMs) {
        TrajectoryDuration returnValue = TrajectoryDuration.Trajectory_Duration_0ms;
        returnValue = returnValue.valueOf(durationMs);
        if (returnValue.value != durationMs) {
            System.out.println("Error: Motion profile duration time not valid");
        }

        return returnValue;
    }

    private void startFillingPoints(double[][] profileLeft,
            double[][] profileRight, int totalCount) {
        System.out.println("Starting filling MP points");

        TrajectoryPoint pointLeft = new TrajectoryPoint();
        TrajectoryPoint pointRight = new TrajectoryPoint();

        if (statusLeft.hasUnderrun) {
            System.out.println("Error: Motion Profile Left has underrun.");
            talonLeft.clearMotionProfileHasUnderrun(0);
        }
        if (statusRight.hasUnderrun) {
            System.out.println("Error: Motion Profile Right has underrun.");
            talonRight.clearMotionProfileHasUnderrun(0);
        }

        talonLeft.clearMotionProfileTrajectories();
        talonRight.clearMotionProfileTrajectories();

        talonLeft.configMotionProfileTrajectoryPeriod(0, 0);
        talonRight.configMotionProfileTrajectoryPeriod(0, 0);

        for (int i = 0; i < totalCount; ++i) {
            double positionFeetLeft = profileLeft[i][0];
            double velocityFPSLeft = profileLeft[i][1];
            /* for each point, fill our structure and pass it to API */
            pointLeft.position = 12 * positionFeetLeft 
                    / (2 * Math.PI * WHEEL_RADIUS) * TICKS_PER_REVOLUTION; // Convert to Units
            pointLeft.velocity = 12 * 60 * velocityFPSLeft 
                    / (2 * Math.PI * WHEEL_RADIUS) 
                    * TICKS_PER_REVOLUTION / 600.0; // Convert to Units/100ms
            pointLeft.headingDeg = 0; /* future feature - not used in this example */
            pointLeft.profileSlotSelect0 = 1; /* which set of gains would you like to use [0,3]? */
            pointLeft.timeDur = getTrajectoryDuration((int) profileLeft[i][2]);
            pointLeft.zeroPos = false;

            System.out.println("Pos " + pointLeft.position);
            System.out.println("Vel " + pointLeft.velocity);
            System.out.println("Dur " + pointLeft.timeDur);

            System.out.println("");

            if (i == 0) {
                pointLeft.zeroPos = true; /* set this to true on the first point */
            }

            pointLeft.isLastPoint = false;
            if ((i + 1) == totalCount) {
                pointLeft.isLastPoint = true; /* set this to true on the last point */
            }

            talonLeft.pushMotionProfileTrajectory(pointLeft);

            // Do the same for the Right side
            double positionFeetRight = profileRight[i][0];
            double velocityFPSRight = profileRight[i][1];
            /* for each point, fill our structure and pass it to API */
            pointRight.position = 12 * positionFeetRight 
                    / (2 * Math.PI * WHEEL_RADIUS) * TICKS_PER_REVOLUTION;
            pointRight.velocity = 12 * 60 * velocityFPSRight 
                    / (2 * Math.PI * WHEEL_RADIUS) * TICKS_PER_REVOLUTION / 600.0;
            pointRight.headingDeg = 0;
            pointRight.profileSlotSelect0 = 1;
            pointRight.timeDur = getTrajectoryDuration((int) profileRight[i][2]);
            pointRight.zeroPos = false;

            if (i == 0) {
                pointRight.zeroPos = true;
            }

            pointRight.isLastPoint = false;
            if ((i + 1) == totalCount) {
                pointRight.isLastPoint = true; /* set this to true on the last point */
            }

            talonRight.pushMotionProfileTrajectory(pointRight);

        }

    }

    public void startMotionProfile() {
        start = true;
    }

    public SetValueMotionProfile getSetValue() {
        return setValue;
    }

    public boolean isMotionProfileComplete() {
        return finished;
    }
    class PeriodicRunnable implements java.lang.Runnable {
        @Override
        public void run() {
            talonRight.processMotionProfileBuffer();
            talonLeft.processMotionProfileBuffer();
        }
    }
}