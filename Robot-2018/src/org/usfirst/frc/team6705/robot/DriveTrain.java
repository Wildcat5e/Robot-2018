package org.usfirst.frc.team6705.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static org.usfirst.frc.team6705.robot.Constants.*;

public class DriveTrain {

	static WPI_TalonSRX leftTalon = new WPI_TalonSRX(leftTalonChannel);
	static WPI_TalonSRX rightTalon = new WPI_TalonSRX(rightTalonChannel);
	static WPI_VictorSPX leftVictor = new WPI_VictorSPX(leftVictorChannel);
	static WPI_VictorSPX rightVictor = new WPI_VictorSPX(rightVictorChannel);
	
	static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	private static int turningStableTicks = 0;
	private static double previousTurningError = 0;
	private static double turningIntegral = 0;
	private static int endMoveTicks = 0;
	private static int timeOutTicks = 0;
	
	public static void setup() {
		leftVictor.follow(leftTalon);
		rightVictor.follow(rightTalon);
		
		//leftTalon.setSafetyEnabled(true);
		//rightTalon.setSafetyEnabled(true);
		
		leftTalon.setNeutralMode(NeutralMode.Brake);
		rightTalon.setNeutralMode(NeutralMode.Brake);
		leftVictor.setNeutralMode(NeutralMode.Brake);
		rightVictor.setNeutralMode(NeutralMode.Brake);
		
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
		leftVictor.setInverted(true);
		rightVictor.setInverted(false);
		
		leftTalon.configOpenloopRamp(rampRateTeleop, 0);
		rightTalon.configOpenloopRamp(rampRateTeleop, 0);
		leftTalon.configClosedloopRamp(rampRateAuto, 0);
		rightTalon.configClosedloopRamp(rampRateAuto, 0);
		
		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		
		leftTalon.setSensorPhase(true);
		rightTalon.setSensorPhase(true);
		
		configPID();
		
		rightTalon.configMotionProfileTrajectoryPeriod(10, 0);
		leftTalon.configMotionProfileTrajectoryPeriod(10, 0);
		rightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		leftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		
		gyro.reset();
		resetEncoders();
	}
	
	public static void configPID() {
		//Velocity PID in slot 0
		leftTalon.config_kP(0, kP_L, 0);
		leftTalon.config_kI(0, kI, 0);
		leftTalon.config_kD(0, kD, 0);
		leftTalon.config_kF(0, kF_L, 0);
		rightTalon.config_kP(0, kP_R, 0);
		rightTalon.config_kI(0, kI, 0);
		rightTalon.config_kD(0, kD, 0);
		rightTalon.config_kF(0, kF_R, 0);
		
		//Motion profile PID in slot 1
		leftTalon.config_kP(1, kP_MP, 0);
		leftTalon.config_kI(1, kI_MP, 0);
		leftTalon.config_kD(1, kD_MP, 0);
		leftTalon.config_kF(1, kF_MP, 0);
		rightTalon.config_kP(1, kP_MP, 0);
		rightTalon.config_kI(1, kI_MP, 0);
		rightTalon.config_kD(1, kD_MP, 0);
		rightTalon.config_kF(1, kF_MP, 0);
	}
	
	public static void reverseDriveTrain() {
		System.out.print("REVERSED DRIVE TRAIN");
		leftTalon.setInverted(false);
		rightTalon.setInverted(true);
		leftVictor.setInverted(false);
		rightVictor.setInverted(true);
	}
	
	public static void undoReverseDriveTrain() {
		System.out.print("RESET REVERSED DRIVE TRAIN");
		leftTalon.setInverted(true);
		rightTalon.setInverted(false);
		leftVictor.setInverted(true);
		rightVictor.setInverted(false);
	}
	
	public static void switchToVelocityMode() {
		leftTalon.selectProfileSlot(0, 0);
		rightTalon.selectProfileSlot(0, 0);
	}
	
	public static void switchToMotionProfile() {
		leftTalon.selectProfileSlot(1, 0);
		rightTalon.selectProfileSlot(1, 0);
	}
	
	//Tank drive for teleop control
	public static void tankDrive(double leftSpeed, double rightSpeed) {
		leftSpeed = applyDeadband(leftSpeed);
		rightSpeed = applyDeadband(rightSpeed);
		
		
		double leftTarget = Math.copySign(leftSpeed * leftSpeed * leftSpeed * -1, leftSpeed) * maxTicksPer100ms;
		double rightTarget = Math.copySign(rightSpeed * rightSpeed * rightSpeed * -1, rightSpeed) * maxTicksPer100ms;
		
		//System.out.print("Speed Left:" + getFPS(leftTarget));
		//System.out.println("Speed right:" + getFPS(rightTarget));
		
		setVelocity(leftTarget, rightTarget);
		
		//Percent Output Mode
		//setSpeed(leftSpeed * leftSpeed * leftSpeed, rightSpeed * rightSpeed * rightSpeed);

	}
	
	public static double applyDeadband(double speed) {
		if ((speed < deadband && speed > 0) || (speed > -deadband && speed < 0)) {
			speed = 0;
		} else if (speed > 0.92) {
			speed = 1;
		} else if (speed < -0.92) {
			speed = -1;
		}
		return speed;
	}

	//Autonomous move method
	public static boolean moveByDistance(double inches, double degrees, double velocity, double timeOutSeconds) {
		double targetEncoderTicks = Math.abs(convertInchesToTicks(inches));
		double ticksSoFar = Math.abs((leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0))/2);
		double maxVelocity = convertVelocity(velocity);
				
		if (ticksSoFar >= targetEncoderTicks) {
			System.out.println("DONE WITH MOVE BY DISTANCE");
			endMoveTicks++;
			DriveTrain.stop();
			
			if (endMoveTicks > 5) { //Pause very briefly after the move
				//resetEncoders();
				//gyro.reset();
				Robot.auto.previousFinalTurningError = 0;
				endMoveTicks = 0;
				return true;
			} else {
				return false;
			}
		} else if (leftTalon.getSelectedSensorVelocity(0) < 10 && rightTalon.getSelectedSensorVelocity(0) < 10 && timeOutSeconds > 0) {
			timeOutTicks++;
			if (timeOutTicks > timeOutSeconds * 50) {
				DriveTrain.stop();
				Robot.auto.previousFinalTurningError = 0;
				endMoveTicks = 0;
				timeOutTicks = 0;
				return true;
			}
		} else {
			timeOutTicks = 0;
		}
		
		int direction = (inches > 0) ? 1 : -1;
		
		double ticksRemaining = targetEncoderTicks - ticksSoFar;
		//System.out.println("Ticks Remaining: " + ticksRemaining);
		double fractionRemaining = ticksRemaining/targetEncoderTicks;
		double scaledFraction = fractionRemaining * 3; //Start slowing down 2/3 of the way there
		if (scaledFraction > 1) {
			scaledFraction = 1;
		}
		
		double correctedHeading = degrees + Robot.auto.previousFinalTurningError;
		
		double degreeErrorLeft = (getGyro() - correctedHeading > 0) ? getGyro() - correctedHeading : 0;
		double degreeErrorRight = (getGyro() - correctedHeading < 0) ? correctedHeading - getGyro() : 0;

		double velocityLeft = maxVelocity + (kP_StraightDriving * degreeErrorLeft * direction);
		double velocityRight = maxVelocity + (kP_StraightDriving * degreeErrorRight * direction);
		
		double scaledSpeedR = scaledFraction * velocityRight;
		double scaledSpeedL = scaledFraction * velocityLeft;
		
		if (scaledSpeedR < minimumSpeed + (kP_StraightDriving * degreeErrorRight * direction)) {
			scaledSpeedR = minimumSpeed + (kP_StraightDriving * degreeErrorRight * direction);
		}
		if (scaledSpeedL < minimumSpeed + (kP_StraightDriving * degreeErrorLeft * direction)) {
			scaledSpeedL = minimumSpeed  + (kP_StraightDriving * degreeErrorLeft * direction);
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
	

	//Autonomous turn method
	public static boolean turnDegrees(double degrees, double timeOutDegreeTolerance) {
		//Positive degrees -> counterclockwise; negative degrees -> clockwise
		

		double currentAngle = getGyro();
		double error = degrees - currentAngle;
		double absoluteError = Math.abs(error);
		boolean inTolerance = false;
		
		int turnMultiplier = (error < 0) ? -1 : 1;
		
		if (previousTurningError == 0) {
			previousTurningError = degrees;
		}
		
		System.out.println("Turning with current gyro angle " + getGyro() + " and target " + degrees + " and error " + error + " and previous error " + previousTurningError);
		
		if (absoluteError <= turningTolerance) {
			DriveTrain.stop();
			inTolerance = true;
			
			//System.out.println("Has been stable within target's tolerance for " + turningStableTicks + " iterations");
			System.out.println("Within tolerance with velocities L: " + leftTalon.getSelectedSensorVelocity(0) + " and R: " + rightTalon.getSelectedSensorPosition(0));
			
			if (leftTalon.getSelectedSensorVelocity(0) < 75 && rightTalon.getSelectedSensorVelocity(0) < 75) {
				System.out.println("STOP TURNING AT ANGLE: " + getGyro() + " with absolute error " + absoluteError);

				Robot.auto.previousFinalTurningError = error;
				
				DriveTrain.stop();
				//gyro.reset();
		
				turningStableTicks = 0;
				previousTurningError = 0;
				turningIntegral = 0;

				return true;
			}
		} else if (leftTalon.getSelectedSensorVelocity(0) < 25 && rightTalon.getSelectedSensorVelocity(0) < 25 && absoluteError < timeOutDegreeTolerance && timeOutDegreeTolerance != 0) {
			turningStableTicks++;
			if (turningStableTicks > 100) { 
				turningStableTicks = 0;
				previousTurningError = 0;
				turningIntegral = 0;
				
				Robot.auto.previousFinalTurningError = error;
				
				DriveTrain.stop();
				return true;
			}
		} else {
			turningStableTicks = 0;
		}
		
		if (absoluteError < 1) {
			turningIntegral = 0; //Reset the integral to 0 if we are overshooting
		}

		double bias = minimumTurningOutput * turnMultiplier;
		double proportional = error * kP_Turning;
		double derivative = (error - previousTurningError) * kD_Turning;
		
		previousTurningError = error; //Reset previous error to current error
		
		double output =  proportional + derivative + bias;
		if (absoluteError < iZone) {
			turningIntegral += error;
			output += turningIntegral * kI_Turning;
		} else {
			turningIntegral = 0;
		}
		
		System.out.println("Proportional: " + proportional + " integral: " + turningIntegral + " derivative: " + derivative);
		
		if (Elevator.getCurrentPosition() > 50) {
			output *= (1 - (Elevator.getCurrentPosition()/(3.5 * scaleHeight))); //Reduce output with elevator at high heights
		}

		if (Math.abs(output) > maxTurningOutput) {
			output = maxTurningOutput * turnMultiplier;
		} /*else if (Math.abs(output) < minimumTurningOutput) {
			output = minimumTurningOutput * turnMultiplier;
		}*/
		
		if (!inTolerance) {
			System.out.println("Setting turning speed: " + output + " with direction " + turnMultiplier);
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

	//***************************************//
	
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
		leftTalon.set(ControlMode.MotionProfile, setValue.value);
		rightTalon.set(ControlMode.MotionProfile, setValue.value);
		profile.periodic();
		
		if (profile.isMotionProfileComplete()) {
			return true;
		}
		return false;
	}
	
	//***************************************//
	
	public static void setVelocity(double left, double right) {
	    double elevatorHeight = Elevator.encoder.get();
	    double scale = 1; 
	    if (Elevator.getCurrentPosition() > 40) {
	        scale = 1 - (Elevator.getCurrentPosition()/(maximumHeight * 2.7));
	    }
		
		leftTalon.set(ControlMode.Velocity, left * scale);
		rightTalon.set(ControlMode.Velocity, right * scale);
		
		//System.out.println("Setting velocities L: " + left + " R: " + right);
		//System.out.println("Actual speed L: " + leftTalon.getSelectedSensorVelocity(0) + " R: " + rightTalon.getSelectedSensorVelocity(0));
	}
	
	public static void setSpeed(double left, double right) {
		leftTalon.set(ControlMode.PercentOutput, left);
		rightTalon.set(ControlMode.PercentOutput, right);
	}
	
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
		//leftVictor.set(ControlMode.PercentOutput, 0);
		//rightVictor.set(ControlMode.PercentOutput, 0);
	}
	
	
	public static boolean wait(double time, double previousTime) {
		if (Robot.timer.get() - previousTime >= time) {
			return true;
		}
		return false;
	}
	
	public static double getGyro() {
		return -gyro.getAngle();
	}
	
	public static void resetEncoders() {
		leftTalon.setSelectedSensorPosition(0, 0, 0);
		rightTalon.setSelectedSensorPosition(0, 0, 0);
	}
	
}
