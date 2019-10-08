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

    public DcMotor getIl() {
        return il;
    }

    public void setIl(DcMotor il) {
        this.il = il;
    }

    public DcMotor getIr() {
        return ir;
    }

    public void setIr(DcMotor ir) {
        this.ir = ir;
    }

    public DcMotor getLl() {
        return ll;
    }

    public void setLl(DcMotor ll) {
        this.ll = ll;
    }

    public DcMotor getLr() {
        return lr;
    }

    public void setLr(DcMotor lr) {
        this.lr = lr;
    }

    public Servo getGr() {
        return gr;
    }

    public void setGr(Servo gr) {
        this.gr = gr;
    }

    public Servo getGl() {
        return gl;
    }

    public void setGl(Servo gl) {
        this.gl = gl;
    }

    private DcMotor lr; //lift right
    private Servo gr; // gantry right
    private Servo gl; // gantry left
    private CRServo armRotater;
    private Servo pincher;
    private final double TIME_FOR_INTAKE = 3;
    private final double ENCODER_LIFT_UP = 2;  //TODO: Test times for Lift Times
    private final double ENCODER_LIFT_DOWN = 1.5;
    private final double CLOSED_PINCHER_SERVO_POSITION = 0; //TODO: Test for Servo Positions
    private final double OPEN_PICHER_SERVO_POSITION = 90;
    private final double INCHES_TO_SERVO = 0;//TODO: Test conversions for inches in gantry movement to servo position

    public Stacker(LinearOpMode opMode) {
        il = this.opMode.hardwareMap.dcMotor.get("il");
        ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode.hardwareMap.dcMotor.get("ll");
        lr = this.opMode.hardwareMap.dcMotor.get("lr");
        armRotater = this.opMode.hardwareMap.crservo.get("armRotater");
        armRotater.setDirection(DcMotorSimple.Direction.FORWARD);
        pincher = this.opMode.hardwareMap.servo.get("pincher");
        gl = this.opMode.hardwareMap.servo.get("gl");
        gr = this.opMode.hardwareMap.servo.get("gr");

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    public void setIntakePower(double power) {
        il.setPower(power);
        ir.setPower(power);
    }

    public void setLiftPower(double power) {
        ll.setPower(power);
        lr.setPower(power);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void intakeTime(double power) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < TIME_FOR_INTAKE) {
            setIntakePower(power);
        }
        setIntakePower(0);
    }

    public void setPincherPosition(boolean close) {
        if (close) {
            pincher.setPosition(CLOSED_PINCHER_SERVO_POSITION);
        } else {
            pincher.setPosition(OPEN_PICHER_SERVO_POSITION);
        }
    }

    public void rotateArmTime(double power, double time) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < time) {
            armRotater.setPower(power);
        }
        armRotater.setPower(0);

    }

    public void setLiftPosition(double power, boolean up) {
        double currentPos = getLiftEncoderAverage();
        if (up) {
            while (currentPos < ENCODER_LIFT_UP) {
                setLiftPower(power);
            }
            setLiftPower(0);
        } else {
            while (currentPos < ENCODER_LIFT_DOWN) {
                setLiftPower(-power);
            }
            setLiftPower(0);
        }
    }

    public void setGantryPosition(double inches) {
        gl.setPosition(inchToServo(inches));
        gr.setPosition(inchToServo(inches));
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public double getLiftEncoderAverage() {
        double counter = 0;
        if (ll.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (lr.getCurrentPosition() == 0) {
            counter += 1;
        }
        return (ll.getCurrentPosition() + lr.getCurrentPosition() / (2 - counter));
    }
    public double inchToServo(double inches){
        return (inches * INCHES_TO_SERVO);
    }
}

