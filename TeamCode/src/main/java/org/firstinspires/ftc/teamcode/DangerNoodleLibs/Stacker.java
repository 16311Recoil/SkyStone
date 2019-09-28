package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import android.graphics.Path;
import android.graphics.Picture;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Stacker {
    private LinearOpMode opMode;
    private DcMotor il; //intake left
    private DcMotor ir; //intake right
    private DcMotor ll; //lift left
    private DcMotor lr; //lift right
    private CRServo armRotater;
    private Servo pincher;
    private final double TIME_FOR_INTAKE = 3;
    private final double ENCODER_LIFT_UP = 2;  //TODO: Test times for Lift Times
    private final double ENCODER_LIFT_DOWN = 1.5;
    private final double CLOSED_PINCHER_SERVO_POSITION = 0; //TODO: Test for Servo Positions
    private final double OPEN_PICHER_SERVO_POSITION = 90;

    public Stacker(LinearOpMode opMode) {
        il = this.opMode.hardwareMap.dcMotor.get("il");
        ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode.hardwareMap.dcMotor.get("ll");
        lr = this.opMode.hardwareMap.dcMotor.get("lr");
        armRotater = this.opMode.hardwareMap.crservo.get("armRotater");
        armRotater.setDirection(DcMotorSimple.Direction.FORWARD);
        pincher = this.opMode.hardwareMap.servo.get("pincher");

    }
    //////////////////////////////////////////////////////////////////////////////////////////////
    public void setIntakePower ( double power){
        il.setPower(power);
        ir.setPower(power);
    }

    public void setLiftPower (double power){
        ll.setPower(power);
        lr.setPower(power);
    }

    public void intakeTime (double power){
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < TIME_FOR_INTAKE) {
            setIntakePower(power);
        }
        setIntakePower(0);
    }

    public void setIntakePosition ( boolean closed){
        if (closed) {
            pincher.setPosition(CLOSED_PINCHER_SERVO_POSITION);
        }
        else {
            pincher.setPosition(OPEN_PICHER_SERVO_POSITION);
        }
    }

<<<<<<< HEAD
    public void rotateArmTime (double power, double time) {
=======
    public void rotateArmTime (double power, double time){
>>>>>>> 19258ab73991c90480f3e320181e4e9c8cda3da4
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            armRotater.setPower(power);
        }
<<<<<<< HEAD
        armRotater.setPower(0);
    }
=======
       armRotater.setPower(0);

>>>>>>> 19258ab73991c90480f3e320181e4e9c8cda3da4

    public void setLiftPosition (double power, boolean up){
        double currentPos = getLiftEncoderAverage();
            if (up) {
            while (currentPos < ENCODER_LIFT_UP){
                setLiftPower(power);
            }
            setLiftPower(0);
        }
        else {
            while (currentPos < ENCODER_LIFT_DOWN){
                setLiftPower(-power);
            }
            setLiftPower(0);
        }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
<<<<<<< HEAD
    public double getLiftEncoderAverage(){
=======
    public double getLiftEncoderAverage() {
>>>>>>> 19258ab73991c90480f3e320181e4e9c8cda3da4
        double counter = 0;
        if (ll.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (lr.getCurrentPosition() == 0) {
            counter += 1;
        }
        return (ll.getCurrentPosition() + lr.getCurrentPosition() / (2 - counter));
    }

}
