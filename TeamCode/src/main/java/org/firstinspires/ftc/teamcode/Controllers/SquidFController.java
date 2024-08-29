package org.firstinspires.ftc.teamcode.Controllers;

import com.arcrobotics.ftclib.controller.PIDFController;


/**
 * A better controller for distance-based feedback control.
 * Follows abstract kinematics of any given motion system (v = sqrt(2ad) + fric).
 * Typically avoid using kf due to its inherent differences for each system.
 */
public class SquidFController extends PIDFController {
    private PIDFController pidf;

    /**
     * Friction correction.
     */
    private double kfric = 0;
    /**
     * General feedforward term.
     */
    private double kf = 0;


    /**
     * The base constructor for the SquiDF controller
     *
     * @param kp
     * @param ki
     * @param kd
     * @param kf
     */
    public SquidFController(double kp, double ki, double kd, double kf) {
        super(kp, ki, kd, 0);
        this.kf = kf;
    }

    public SquidFController(double kp, double ki, double kd, double kf, double kfric) {
        super(kp, ki, kd, 0);
        this.kf = kf;
        this.kfric = kfric;
    }

    @Override
    public double calculate(double pv) {
        double out = super.calculate(pv);
        out += kf;
        out = Math.sqrt(out);
        out += kfric;
        return out;
    }

    public void setF(double kf) {
        this.kf = kf;
    }

    public double getF() {
        return this.kf;
    }

    public void setKfric(double kfric) {
        this.kfric = kfric;
    }

    public double getKfric() {
        return this.kfric;
    }

}
