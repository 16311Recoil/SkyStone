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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Testing.LiftPrototype;


public class Stacker {


    private boolean changeLtrigger;
    private double intakePower;
    private boolean changeLBumper;
    private boolean changeRBumper;

    private enum State{
        DIRECT,
        L_SPEED,
        H_SPEED
    }
    private State currentState;
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
    private boolean changeA2;
    private boolean changeB2;
    private boolean changeDpadLeft2;

    public Stacker(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Stacker Init Started");
        opMode.telemetry.update();
        il = this.opMode.hardwareMap.dcMotor.get("il");
        ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode.hardwareMap.dcMotor.get("ll");
        lr = this.opMode.hardwareMap.dcMotor.get("lr");
        armRotater = this.opMode.hardwareMap.servo.get("armRotater");
        pincher = this.opMode.hardwareMap.servo.get("pincher");
        //gl = this.opMode.hardwareMap.servo.get("gl");
        //gr = this.opMode.hardwareMap.servo.get("gr");

        ll.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);

        opMode.telemetry.addLine("Stacker Init Completed");
        opMode.telemetry.update();
        currentState = State.DIRECT;

    }
    public Stacker(OpMode opMode){
        this.opMode_iterative = opMode;
        opMode_iterative.telemetry.addLine("Stacker Init Started");
        opMode_iterative.telemetry.update();
        il = this.opMode_iterative.hardwareMap.dcMotor.get("il");
        ir = this.opMode_iterative.hardwareMap.dcMotor.get("ir");
        ll = this.opMode_iterative.hardwareMap.dcMotor.get("ll");
        lr = this.opMode_iterative.hardwareMap.dcMotor.get("lr");
        armRotater = this.opMode_iterative.hardwareMap.servo.get("armRotater");
        pincher = this.opMode_iterative.hardwareMap.servo.get("pincher");
        //gl = this.opMode.hardwareMap.servo.get("gl");
        //gr = this.opMode.hardwareMap.servo.get("gr");

        ll.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);
        intakePower = 0;

        opMode_iterative.telemetry.addLine("Stacker Init Completed");
        opMode_iterative.telemetry.update();

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
    public void telePincher(){
        double counter = 0;
        boolean close;
        if ((opMode_iterative.gamepad1.x ^ changeX) || (opMode_iterative.gamepad2.x ^ changeX2)){
            counter ++;
        }
        if ((counter % 2) == 0){
            close = true;
        }
        else{
            close = false;
        }
        setPincherPosition(close);
        changeX = opMode_iterative.gamepad1.x;
        changeX2 = opMode_iterative.gamepad2.x;
    }
    public void teleArm(){
        double counter = 0;
        boolean in;
        if ((opMode_iterative.gamepad2.y ^ changeY2)){
            counter ++;
        }
        if ((counter % 2) == 0){
            in = true;
        }
        else{
            in = false;
        }
        setArmPosition(in);
        changeY2 = opMode_iterative.gamepad2.y;
    }
    private void liftControlD2() {

        if (opMode_iterative.gamepad2.a ^ changeA2){
            currentState = State.L_SPEED;
        }
        else if (opMode_iterative.gamepad2.b ^ changeB2) {
            currentState = State.H_SPEED;
        }
        else if (opMode_iterative.gamepad2.dpad_left ^ changeDpadLeft2)
            currentState = State.DIRECT;

        changeA2 = opMode_iterative.gamepad2.a;
        changeB2 = opMode_iterative.gamepad2.b;
        changeDpadLeft2 = opMode_iterative.gamepad2.dpad_left;

        ll.setPower(opMode_iterative.gamepad2.left_stick_y);
        lr.setPower(opMode_iterative.gamepad2.left_stick_y);
    }
    private void intakeControlD2(){
        if(opMode_iterative.gamepad2.left_bumper ^ changeLBumper && intakePower <= 1) {
            intakePower += 0.1;
        }
        else if(opMode_iterative.gamepad2.right_bumper ^ changeRBumper && intakePower >= -1){
            if(intakePower == 0.1)
                intakePower = 0;
            else
                intakePower -= 0.1;

        }
        opMode_iterative.telemetry.addData("Intake Power: ",intakePower);
        opMode_iterative.telemetry.update();
        double multiplier = calculatePower();
        il.setPower(multiplier * Range.clip(intakePower, -1,1));
        ir.setPower(-multiplier * Range.clip(intakePower, -1,1));

        changeLBumper = opMode_iterative.gamepad2.x;
        changeRBumper = opMode_iterative.gamepad2.y;
    }
    public void stackerTeleControl(){
        teleArm();
        telePincher();
        intakeControlD2();
        liftControlD2();
    }
    private double calculatePower(){
        if (currentState == State.DIRECT)
            return 1;
        else if (currentState == State.H_SPEED)
            return 0.5;
        else if (currentState == State.L_SPEED)
            return 0.25;
        return 0;
    }

}

