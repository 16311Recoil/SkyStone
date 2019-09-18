package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

public class PID {
    private double k_i;
    private double k_p;
    private double k_d;
    private double t_sum;
    private double previousError;

    private double target;
    private boolean reset;
    private double t_i;

    private double MAX_SUM = 0.0; // test sum

    public PID() {
        k_i = 0.0;
        k_p = 0.0;
        k_d = 0.0;

        target = 0.0;
    }
    public PID(double k_i, double k_p, double k_d, double target) {
        this.k_i = k_i;
        this.k_p = k_p;
        this.k_d = k_d;

        this.target = target;
        reset = true;
    }
    public void setCoeffs(double k_p, double k_i, double k_d){
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;
    }

    /**
     * @param error - current error;
     * @param currentTime - time @ start of iteration
     * @return
     */
    public double iteration(double error, double currentTime){
        double p;
        double i;
        double d;

        if(reset){
            t_sum = 0.0;
            d = 0.0;
            i = 0.0;
            p = 0.0;
            reset = false;
            previousError = Math.abs(target - error);
        } else {
            double deltaTime = currentTime - t_i;
             p = k_p * error;
             t_sum = 0.5 * (error +  previousError) * deltaTime;
             if (t_sum > MAX_SUM)
                 t_sum = MAX_SUM;// test for maxSum
             i = k_i * t_sum;
             d = k_d * (error - previousError) / deltaTime;

        }
        return(p + i + d);
    }

    public double getK_i() {
        return k_i;
    }

    public void setK_i(double k_i) {
        this.k_i = k_i;
    }

    public double getK_p() {
        return k_p;
    }

    public void setK_p(double k_p) {
        this.k_p = k_p;
    }

    public double getK_d() {
        return k_d;
    }

    public void setK_d(double k_d) {
        this.k_d = k_d;
    }



    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }


    public double getT_i() {
        return t_i;
    }

    public void setT_i(double t_i) {
        this.t_i = t_i;
    }

    public boolean isReset() {
        return reset;
    }

    public void setReset(boolean reset) {
        this.reset = reset;
    }
    public void setMAX_SUM(double maxSum){
        MAX_SUM = maxSum;
    }
}
