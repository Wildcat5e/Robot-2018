package org.usfirst.frc.team6705.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class PIDMotor {
	
	public Spark motor;
    private PIDController controller;
    private double rampBand;

    /**
     * Constructor for a PID controlled motor, with a controllable multiplier.
     *
     * @param motor The motor being set.
     * @param rampBand The acceptable range for a motor change in one loop
     * @param controller The PIDController this was passed as output to
     */
    public PIDMotor(Spark motor, double rampBand, PIDController controller) {
            this.motor = motor;
            this.controller = controller;
            this.rampBand = rampBand;
            controller.setOutputRange(0 - rampBand, 0 + rampBand);
    }

    public void pidWrite(double output) {
            motor.set(output);
            controller.setOutputRange(output - rampBand, output + rampBand);
    }
    
}
