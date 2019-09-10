package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    public void turn(double power)
    {

    }
    // needs to be edited; will go over with Aditya!
    public void turnGyro (double power, double target) {
        int angle = 0; // Replacement for getting gyro angles
        if (target > 0) {           // Boolean to check which direction the turn occurs and set motors accordingly
            while (angle < target) {      //left turn
                fl.setPower(power);
                fr.setPower(-power);
                bl.setPower(power);
                br.setPower(-power);
            }
        }
        else {
            while (angle > target) {   //right turn
                fl.setPower(-power);
                fr.setPower(power);
                bl.setPower(-power);
                br.setPower(power);
            }
        }
    }
    public double encoderAverage (){
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
        int currentPos = 0; //replacement for current encoder measurement
        int timer = 0; // placeholder for timer
        startMotors(power); // starts motor
        while  (timer < timeout) {      //timer loop, to stop motors after time reaches timeout
            if (currentPos >= encoderDistance) { //checks if destination was reached, and if so stops motors
                stopMotors();
            }
        }
        stopMotors();
    }
}
