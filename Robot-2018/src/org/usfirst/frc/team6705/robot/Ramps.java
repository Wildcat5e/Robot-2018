package org.usfirst.frc.team6705.robot;

import static org.usfirst.frc.team6705.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Ramps {

    private static DoubleSolenoid leftSolenoid = 
            new DoubleSolenoid(LEFT_RAMP_SOLENOID_A, LEFT_RAMP_SOLENOID_B);
    private static DoubleSolenoid rightSolenoid = 
            new DoubleSolenoid(RIGHT_RAMP_SOLENOID_A, RIGHT_RAMP_SOLENOID_B);

    public static void deploy() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward);
        rightSolenoid.set(DoubleSolenoid.Value.kForward);
    }
}