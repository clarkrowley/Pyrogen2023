package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * PID Control for things.
 * KP of -0.07 seems to be the limit.
 * KD of -0.02 causes wild oscillations, -0.01 is nice.
 * For Driving:
 * SimplePID drivePID = new SimplePID(-0.005,0,0);
 * Good coefficients for steering are:
 *     public double KP = -0.05;
 *     public double KI = -0.001;
 *     public double KD = 0.002;
 */
public class SimplePID {
    //  KP of 0 does not fix crazy oscillations
    //public double KP = -0.06;
    public double KP = -0.05;

    public double KI = -0.001;
    public double KD = 0.002;
    //public double KD = -0.01;

    public double deltaTime = 0.0;
    public double lastTime = 0.0;

    public double error = 0.0;
    public double lastError = 0.0;
    public double errorRate = 0.0;

    public double integral = 0.0;
    public double lastIntegral = 0.0;
    public double derivative = 0.0;

    public double output = 0.0;

    public double bias = 0.0;
    public boolean isChanging = true;
    public double errorMargin = 10.0;
    public double derivativeThresh = 1;
    boolean started = false;
    ElapsedTime runtime;
    /**
     * Create a new SimplePID object with the specified PID values
     * @param kp - Proportional coefficient
     * @param ki - Integral coefficient
     * @param kd - Derivative coefficient
     */
    public SimplePID(double kp, double ki, double kd) {
        this.KP = kp;
        this.KI = ki;
        this.KD = kd;
    }

    public double update(double error, double time) {
        return update(error);
    }
    public double update(double error) {
        if(!started) {
            started = true;
            runtime = new ElapsedTime();
            lastTime = runtime.time();
            return 0.0;
        }
        this.error = error;

        deltaTime = runtime.time() - lastTime;
        if (KI != 0.0 && runtime.time() > 4.0) {
            integral = lastIntegral + error * deltaTime;
        }

        derivative = (error - lastError) / deltaTime;

        output = KP * error + KI*integral + KD * derivative + bias;
        //utput = KP * error;
        if (KI != 0.0) {
            lastIntegral = integral;
        }
        lastError = error;
        lastTime = runtime.time();
        if(Math.abs(derivative) < derivativeThresh) {
            if(Math.abs(error) < errorMargin) {
                isChanging = false;
            }
        }
        return output;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("", "---------- SimplePID ----------");
        telemetry.addData("output", "%.5f", (float)output);
        telemetry.addData("error", "%.5f", (float)error);
        telemetry.addData("Integral", "%.4f", (float)integral);
        telemetry.addData("Derivative", "%.5f", (float)derivative);
    }

    public void reset() {
        deltaTime = 0.0;
        lastTime = 0.0;
        error = 0.0;
        lastError = 0.0;
        errorRate = 0.0;
        integral = 0.0;
        lastIntegral = 0.0;
        derivative = 0.0;
        output = 0.0;
        bias = 0.0;
        isChanging = true;
    }

}
