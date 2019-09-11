package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {

    private LinearOpMode opMode;

    // Instance Variables
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        fl = this.opMode.hardwareMap.dcMotor.get("fl");
        fr = this.opMode.hardwareMap.dcMotor.get("fr");
        bl = this.opMode.hardwareMap.dcMotor.get("bl");
        br = this.opMode.hardwareMap.dcMotor.get("br");

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    /* ============================ UTILITY METHODS ==============================================*/
    public void startMotors(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }
    public void stopMotors() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

    }
    public void turn(double power, boolean right) {
        if (right){
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
        }
        else {
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        }
    }
    // ADITYA !!!
    // Modify this method; instead of counting encoders that aren't zero, count encoders that are 0.
    // It will take away one variable.
    public double getEncoderAverage (){
        double average = 0;
        double counter = 0;
        if (fl.getCurrentPosition() != 0){      // Checks whether encoder outputs zero
            average += fl.getCurrentPosition(); // If not zero, adds to average
            counter += 1 ;                      // Counter to tell what to divide by
        }
        if (fr.getCurrentPosition() != 0){      // Repeated for all encoders
            average += fr.getCurrentPosition();
            counter += 1;
        }
        if (bl.getCurrentPosition() != 0) {
            average += bl.getCurrentPosition();
            counter += 1;
        }
        if (br.getCurrentPosition() != 0) {
            average += br.getCurrentPosition();
            counter += 1;
        }
        average /= counter; //The total sum of all encoders that aren't zero, divided by number of encoders that didn't output zero
        return average;

    }
    public void moveForward (double encoderDistance, double power, double timeout){
        double currentPos = getEncoderAverage();
        ElapsedTime timer = new ElapsedTime();
        while  (timer.seconds() < timeout && currentPos < encoderDistance) { //timer loop to stop motors after time reaches timeout or if destination is reached
            startMotors(power);
        }
        stopMotors();
    }
    /* ============================ MOVEMENT METHODS =============================================*/

    public void turnGyro (double power, double target, boolean right) {
        int angle = 0; // Replacement for getting gyro angles
        while (angle < target && right) {
            turn(power, true)
        }
        while (!right && angle < target) {
            turn(power, false);
        }
    }
    /**
     *  PID move straight method:
     *  Feedback using gyro.
     *  Target is always 0.
     */
    public void movePID(int encoderDistance, double k_p, double k_i, double k_d, double correction, int timeout){
        ElapsedTime t_i = new ElapsedTime();



    }
    /**
     *  PID turning:
     *  Feedback using gyro.
     *  Target is always 0.
     */
    public void turnPID(double targetAngle, double k_p, double k_i, double k_d, double bias, int timeout){
        ElapsedTime time;
    }


}
