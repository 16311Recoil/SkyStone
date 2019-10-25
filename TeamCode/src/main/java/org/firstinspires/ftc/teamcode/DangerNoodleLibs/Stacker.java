package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import android.graphics.Path;
import android.graphics.Picture;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Testing.LiftPrototype;


public class Stacker {
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private DcMotor il; //intake left
    private DcMotor ir; //intake right
    private DcMotor ll; //lift left
    private DcMotor lr; //lift right
    private Servo gr; // gantry right
    private Servo gl; // gantry left
    private Servo armRotater;
    private Servo pincher;
    private boolean changeX;
    private boolean changeY;
    private boolean changeX2;
    private boolean changeY2;
    private final double TIME_FOR_INTAKE = 3;
    private final double ENCODER_LIFT_UP = 2;  //TODO: Test times for Lift Times
    private final double ENCODER_LIFT_DOWN = 1.5;
    private final double CLOSED_PINCHER_SERVO_POSITION = 0; //TODO: Test for Servo Positions
    private final double OPEN_PICHER_SERVO_POSITION = 90;
    private final double ARM_IN = 0;
    private final double ARM_OUT = 1;
    private final double INCHES_TO_SERVO = 0;//TODO: Test conversions for inches in gantry movement to servo position

    public Stacker(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Stacker Init Started");
        opMode.telemetry.update();
        //il = this.opMode.hardwareMap.dcMotor.get("il");
        //ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode.hardwareMap.dcMotor.get("ll");
        lr = this.opMode.hardwareMap.dcMotor.get("lr");
        //armRotater = this.opMode.hardwareMap.servo.get("armRotater");
        //pincher = this.opMode.hardwareMap.servo.get("pincher");
        //gl = this.opMode.hardwareMap.servo.get("gl");
        //gr = this.opMode.hardwareMap.servo.get("gr");

        opMode.telemetry.addLine("Stacker Init Completed");
        opMode.telemetry.update();

    }
    public Stacker(OpMode opMode){
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Stacker Init Started");
        opMode_iterative.telemetry.update();
        //il = this.opMode.hardwareMap.dcMotor.get("il");
        //ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode_iterative.hardwareMap.dcMotor.get("ll");
        lr = this.opMode_iterative.hardwareMap.dcMotor.get("lr");
        //armRotater = this.opMode.hardwareMap.servo.get("armRotater");
        //pincher = this.opMode.hardwareMap.servo.get("pincher");
        //gl = this.opMode.hardwareMap.servo.get("gl");
        //gr = this.opMode.hardwareMap.servo.get("gr");

        opMode.telemetry.addLine("Stacker Init Completed");
        opMode.telemetry.update();

    }

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


    public void setArmPosition(boolean in) {
        if (in){
            armRotater.setPosition(ARM_IN);
        }
        else{
            armRotater.setPosition(ARM_OUT);
        }
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
    ////////////////////////////////////////////////////////////////////////////////////
    //          Tele-Op
    public void teleLiftTrigger(){
        if ((opMode.gamepad1.right_trigger == 0) && (opMode.gamepad1.left_trigger == 0))
            setLiftPower(0);
        if (opMode.gamepad1.right_trigger != 0) {
            setLiftPower(opMode.gamepad1.right_trigger);
        }
        else if (opMode.gamepad1.left_trigger != 0);{
            setLiftPower(opMode.gamepad1.left_trigger);
        }
    }
    public void teleLiftBumpers(double power){
        if (opMode.gamepad1.right_bumper || opMode.gamepad2.right_bumper){
            setLiftPower(power);
        }
        else if (opMode.gamepad1.left_bumper || opMode.gamepad2.left_bumper){
            setLiftPower(-power);
        }
        else {
            setLiftPower(0);
        }
    }
    public void teleIntake(double power){
        if (opMode.gamepad1.a || opMode.gamepad2.a){
            setIntakePower(power);
        }
        else if (opMode.gamepad1.b || opMode.gamepad2.b){
            setIntakePower(-power);
        }
        else {
            setIntakePower(0);
        }
    }
    public void telePincher(){
        double counter = 0;
        boolean close;
        if ((opMode.gamepad1.x ^ changeX) || (opMode.gamepad2.x ^ changeX2)){
            counter ++;
        }
        if ((counter % 2) == 0){
            close = true;
        }
        else{
            close = false;
        }
        setPincherPosition(close);
        changeX = opMode.gamepad1.x;
        changeX2 = opMode.gamepad2.x;
    }
    public void teleArm(){
        double counter = 0;
        boolean in;
        if ((opMode.gamepad1.y ^ changeY) || (opMode.gamepad2.y ^ changeY2)){
            counter ++;
        }
        if ((counter % 2) == 0){
            in = true;
        }
        else{
            in = false;
        }
        setArmPosition(in);
        changeY = opMode.gamepad1.y;
        changeY2 = opMode.gamepad2.y;
    }
    /*
    private void liftControlD2() {

        if (gamepad2.a ^ changeA2){
            currentState = LiftPrototype.State.L_SPEED;
        }
        else if (gamepad2.b ^ changeB2) {
            currentState = LiftPrototype.State.H_SPEED;
        }
        else if (gamepad2.dpad_left ^ changeDpadLeft2)
            currentState = LiftPrototype.State.DIRECT;

        changeA = gamepad2.a;
        changeB = gamepad2.b;
        changeDpadLeft2 = gamepad2.dpad_left;

        ll.setPower(gamepad2.left_stick_y);
        lr.setPower(gamepad2.left_stick_y);
    }
    */

}

