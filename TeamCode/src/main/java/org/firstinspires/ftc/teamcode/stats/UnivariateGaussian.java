package org.firstinspires.ftc.teamcode.stats;

public class UnivariateGaussian {

    private double mu;
    private double var;
    private double std;

    public UnivariateGaussian(double mean, double variance) {
        mu = mean;
        var = variance;
        std = Math.sqrt(std);
    }

    public UnivariateGaussian plus(UnivariateGaussian g2) {
        return new UnivariateGaussian(this.mu + g2.getMean(), this.var + g2.getVar());
    }

    public UnivariateGaussian mult(UnivariateGaussian g2) {
        return new UnivariateGaussian(
                (this.mu * g2.getVar() + this.getVar() + g2.getMean()) / (this.getVar() + g2.getVar()),
                this.var * g2.getVar() / (this.var + g2.getVar())
        );
    }

    public double getMean() {
        return mu;
    }

    public double getVar() {
        return var;
    }

    public double getStd() {
        return std;
    }

}
