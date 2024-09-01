package org.firstinspires.ftc.teamcode.stats.filtering;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.data.Matrix;
import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.stats.MultivariateGaussian;

public class MultivariateKalman {

    /**
     * State + state covariances.
     */
    private MultivariateGaussian x;
    /**
     * Process noise matrix.
     */
    private SimpleMatrix Q;
    /**
     * Measurement function: takes state and outputs a vector in measurement space
     */
    private SimpleMatrix H;
    /**
     * Input function: takes input and transforms it into state space
     */
    private SimpleMatrix B;

    private ElapsedTime time;


    /**
     *
     * @param x Initial state.
     * @param P Initial state covariances.
     * @param Q Process noise matrix.
     * @param H Measurement function: takes state and outputs a vector in measurement space.
     * @param B Input function: takes input and transforms it into state space.
     */
    public MultivariateKalman(SimpleMatrix x, SimpleMatrix P, SimpleMatrix Q, SimpleMatrix H, SimpleMatrix B) {
        this.x = new MultivariateGaussian(x, P);
        this.Q = Q;
        this.H = H;
        this.B = B;
    }

    /**
     * @param F State transformation function: takes state and outputs a vector in state space, advanced by deltatime.
     */
    public void predict(SimpleMatrix F) {
        predict(SimpleMatrix.filled(x.getMeans().getNumRows(), 1, 0.0));
    }

    /**
     * @param F State transformation function: takes state and outputs a vector in state space, advanced by deltatime.
     * @param u Input vector.
     */
    public void predict(SimpleMatrix F, SimpleMatrix u) {
        this.x.setMeans(F.mult(x.getMeans()).plus(B.mult(u)));
        this.x.setCovariances(F.mult(x.getCovariances()).mult(F.transpose()).plus(Q));
    }

    /**
     * @param z Measurement vector.
     * @param R Measurement covariances.
     */
    public void update(SimpleMatrix z, SimpleMatrix R) {
        // residual
        SimpleMatrix y = z.minus(H.mult(x.getMeans()));

        // measurement space covariance, analogous to process variance + measurement variance in univariate - represents uncertainty in measurement + uncertainty in prediction
        SimpleMatrix S = H.mult(x.getCovariances()).mult(H.transpose()).plus(R);

        // kalman gain - uncertainty in prediction / total uncertainty (S)
        // HTransposed here is to convert back to state space
        SimpleMatrix K = x.getCovariances().mult(H.transpose()).mult(S.invert());

        // prior * (1-K) + z * (K)
        this.x.setMeans(x.getMeans().plus(K.mult(y)));

        // ( 1 - K ) * covariance
        this.x.setCovariances(SimpleMatrix.identity(x.getCovariances().getNumRows()).minus(K.mult(H)).mult(x.getCovariances()));
    }

    public void bulkUpdate(SimpleMatrix F, SimpleMatrix z, SimpleMatrix R) {
        predict(F);
        update(z, R);
    }

    public void bulkUpdate(SimpleMatrix F, SimpleMatrix u, SimpleMatrix z, SimpleMatrix R) {
        predict(F, u);
        update(z, R);
    }

    public SimpleMatrix getMeans() {
        return this.x.getMeans();
    }

}
