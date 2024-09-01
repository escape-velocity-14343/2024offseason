package org.firstinspires.ftc.teamcode.stats;

import org.ejml.simple.SimpleMatrix;

public class MultivariateGaussian {

    private SimpleMatrix means;
    private SimpleMatrix covariances;

    public SimpleMatrix getMeans() {
        return means;
    }

    public void setMeans(SimpleMatrix means) {
        this.means = means;
    }

    public SimpleMatrix getCovariances() {
        return covariances;
    }

    public void setCovariances(SimpleMatrix covariances) {
        this.covariances = covariances;
    }

    public MultivariateGaussian(double[] means, double[][] covariances) {
        this(new SimpleMatrix(means.length, 1, true, means),
                new SimpleMatrix(covariances));
    }

    public MultivariateGaussian(SimpleMatrix means, SimpleMatrix covariances) {
        if (means.getNumRows() != covariances.getNumRows()) {
            throw new IllegalArgumentException("Means and Variances do not have equal length.");
        }

        if (covariances.getNumRows() != covariances.getNumCols()) {
            throw new IllegalArgumentException("Covariance Matrix is not square.");
        }

        this.means = means;
        this.covariances = covariances;
    }

    public MultivariateGaussian plus(MultivariateGaussian g2) {
        return new MultivariateGaussian(g2.getMeans().plus(means), g2.getCovariances().plus(covariances));
    }

    public MultivariateGaussian mult(MultivariateGaussian g2) {
        SimpleMatrix denom = g2.getCovariances().plus(covariances).invert();
        return new MultivariateGaussian(g2.getCovariances().mult(denom).mult(means).plus(covariances.mult(denom).mult(g2.getMeans())),
                covariances.mult(denom).mult(g2.getCovariances()));
    }



}
