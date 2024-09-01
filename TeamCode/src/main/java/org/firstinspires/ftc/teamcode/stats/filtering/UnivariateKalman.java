package org.firstinspires.ftc.teamcode.stats.filtering;

import org.firstinspires.ftc.teamcode.stats.UnivariateGaussian;

public class UnivariateKalman {

    private UnivariateGaussian state;
    private UnivariateGaussian process;

    public UnivariateKalman(double x, double initialVariance, double u, double P) {
        this.state = new UnivariateGaussian(x, initialVariance);
        this.process = new UnivariateGaussian(u, P);
    }

    protected void predict() {
        this.state = state.plus(process);
    }

    protected void internalUpdate(double z, double R) {
        this.state = state.mult(new UnivariateGaussian(z, R));
    }

    public void update(double z, double R) {
        predict();
        internalUpdate(z, R);
    }

    public double getState() {
        return state.getMean();
    }

}
