package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;



public class Drivetrain {

    private LinearOpMode opMode;

    private final double WHEEL_DIAMETER_MM = 100.0;
    private final double WHEEL_DIAMETER_FEET = 0.328084;
    private final double ENCODER_PER_REVOLOUTION = 537.6;

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

    public double getEncoderAverage (){
        double counter = 0;
        if (fl.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (fr.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (bl.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (br.getCurrentPosition() == 0) {
            counter += 1;
        }
        return (fl.getCurrentPosition() + bl.getCurrentPosition() + fr.getCurrentPosition() + br.getCurrentPosition()) / (4 - counter);
    }
    public void moveForward (double distance, double power, double timeout){
        double currentPos = getEncoderAverage();
        ElapsedTime timer = new ElapsedTime();
        while  (timer.seconds() < timeout && currentPos < feetToEncoder(distance)) { //timer loop to stop motors after time reaches timeout or if destination is reached
            startMotors(power);
        }
        stopMotors();
    }
    public double actualPower (double desiredPower, double input){
        return (desiredPower / input);
    }
    public void move (double desiredPower, double bias, double angle){
        fl.setPower((actualPower(desiredPower, Math.sin(angle - Math.PI / 4))) * (Math.sin(angle - Math.PI / 4)) + bias);
        fr.setPower((actualPower(desiredPower, Math.cos(angle - Math.PI / 4))) * (Math.cos(angle - Math.PI / 4)) + bias);
        br.setPower((actualPower(desiredPower, Math.sin(angle - Math.PI / 4))) * (Math.sin(angle - Math.PI / 4)) + bias);
        bl.setPower((actualPower(desiredPower, Math.cos(angle - Math.PI / 4))) * (Math.cos(angle - Math.PI / 4)) + bias);
    }
    public double feetToEncoder (double distance){
        return ENCODER_PER_REVOLOUTION * (distance / WHEEL_DIAMETER_FEET);
    }

    /* ============================ MOVEMENT METHODS =============================================*/

    public void turnGyro (double power, double target, boolean right) {
        int angle = 0; // Replacement for getting gyro angles
        while (angle < target && right) {
            turn(power, true);
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
