package org.usfirst.frc.team6705.robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static org.usfirst.frc.team6705.robot.Constants.*;


import edu.wpi.first.wpilibj.Notifier;

public class MotionProfile {

	private MotionProfileStatus status = new MotionProfileStatus();
	double pos = 0, vel = 0, heading = 0;
	private WPI_TalonSRX talonLeft;
	private WPI_TalonSRX talonRight;
	
	private int state = 0;
	private boolean start = false;
	
	private double[][] profileLeft;
	private double[][] profileRight;
	private int numberOfPoints;
	
	private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;

	private static final int minimumTrajectoryPoints = 5;
	
	private int timeoutLoops = -1;
	private static final int timeoutLimit = 10;
	
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  
	    	talonRight.processMotionProfileBuffer(); 
	    	talonLeft.processMotionProfileBuffer();
	    }
	}
	
	Notifier notifier = new Notifier(new PeriodicRunnable());
	
	public MotionProfile(WPI_TalonSRX left, WPI_TalonSRX right, double[][] arrayLeft, double[][] arrayRight, int totalCount) {
		talonLeft = left;
		talonRight = right;
		
		talonLeft.changeMotionControlFramePeriod(5);
		talonRight.changeMotionControlFramePeriod(5);
		
		notifier.startPeriodic(0.005);
		
		profileLeft = arrayLeft;
		profileRight = arrayRight;

		numberOfPoints = totalCount;
	}
	
	public void reset() {
		talonLeft.clearMotionProfileTrajectories();
		talonRight.clearMotionProfileTrajectories();
		
		setValue = SetValueMotionProfile.Disable;
		state = 0;
		timeoutLoops = -1;
		start = false;
	}
	
	public void periodic() {
		
		talonLeft.getMotionProfileStatus(status);
		talonRight.getMotionProfileStatus(status);

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
			//In Motion Profile Mode
			
			switch (state) {
			case 0:
				//Not started yet
				if (start) {
					start = false;
					setValue = SetValueMotionProfile.Disable;
					
					startFillingPoints(profileLeft, profileRight, numberOfPoints);
					
					state = 1;
					timeoutLoops = timeoutLimit;
				}
				break;
			case 1:
				//Wait for points to start filling
				if (status.btmBufferCnt > minimumTrajectoryPoints) {
					setValue = SetValueMotionProfile.Enable;
					state = 2;
					timeoutLoops = timeoutLimit;
				}
				break;
			case 2:
				if (!status.isUnderrun) {
					timeoutLoops = timeoutLimit;
				}
				
				if (status.activePointValid && status.isLast) {
					//Motion Profile complete, load next one
					setValue = SetValueMotionProfile.Hold;
					state = 0;
					timeoutLoops = -1;
				}
				break;
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
	
	private void startFillingPoints(double[][] profileLeft, double[][] profileRight, int totalCount) {
		TrajectoryPoint pointLeft = new TrajectoryPoint();
		TrajectoryPoint pointRight = new TrajectoryPoint();
		
		if (status.hasUnderrun) {
			System.out.println("Error: Motion Profile has underrun.");
			talonLeft.clearMotionProfileHasUnderrun(0);
			talonRight.clearMotionProfileHasUnderrun(0);
		}
		
		talonLeft.clearMotionProfileTrajectories();
		talonRight.clearMotionProfileTrajectories();
		
		talonLeft.configMotionProfileTrajectoryPeriod(0, 0);
		talonRight.configMotionProfileTrajectoryPeriod(0, 0);

		for (int i = 0; i < totalCount; ++i) {
			double positionRotationsLeft = profileLeft[i][0];
			double velocityRPMLeft = profileLeft[i][1];
			/* for each point, fill our structure and pass it to API */
			pointLeft.position = positionRotationsLeft * ticksPerRevolution; //Convert Revolutions to Units
			pointLeft.velocity = velocityRPMLeft * ticksPerRevolution / 600.0; //Convert RPM to Units/100ms
			pointLeft.headingDeg = 0; /* future feature - not used in this example*/
			pointLeft.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			pointLeft.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			pointLeft.timeDur = getTrajectoryDuration((int)profileLeft[i][2]);
			pointLeft.zeroPos = false;
			
			if (i == 0)
				pointLeft.zeroPos = true; /* set this to true on the first point */

			pointLeft.isLastPoint = false;
			if ((i + 1) == totalCount)
				pointLeft.isLastPoint = true; /* set this to true on the last point  */

			talonLeft.pushMotionProfileTrajectory(pointLeft);
			
			//Do the same for the Right side
			double positionRotationsRight = profileRight[i][0];
			double velocityRPMRight = profileRight[i][1];
			/* for each point, fill our structure and pass it to API */
			pointRight.position = positionRotationsRight * ticksPerRevolution;
			pointRight.velocity = velocityRPMRight * ticksPerRevolution / 600.0; 
			pointRight.headingDeg = 0; 
			pointRight.profileSlotSelect0 = 0; 
			pointRight.profileSlotSelect1 = 0; 
			pointRight.timeDur = getTrajectoryDuration((int)profileRight[i][2]);
			pointRight.zeroPos = false;
			
			if (i == 0)
				pointRight.zeroPos = true;

			pointRight.isLastPoint = false;
			if ((i + 1) == totalCount)
				pointRight.isLastPoint = true; /* set this to true on the last point  */

			talonRight.pushMotionProfileTrajectory(pointRight);
			
		}
		
	}
	
	public void startMotionProfile() {
		start = true;
	}
	
	public SetValueMotionProfile getSetValue() {
		return setValue;
	}
	
	
}