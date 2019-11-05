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
    private CRServo gr; // gantry right
    private CRServo gl; // gantry left
    private Servo armRotater;
    private Servo pincher;
    private Servo lFang;
    private Servo rFang;
    private final double TIME_FOR_INTAKE = 3;
    private final double ENCODER_LIFT_UP = 2;  //TODO: Test times for Lift Times
    private final double ENCODER_LIFT_DOWN = 1.5;
    private final double CLOSED_PINCHER_SERVO_POSITION = 0; //TODO: Test for Servo Positions
    private final double OPEN_PICHER_SERVO_POSITION = 90;
    private final double ARM_IN = 0;
    private final double ARM_OUT = 1;
    private final double INCHES_TO_SERVO = 0;//TODO: Test conversions for inches in gantry movement to servo position
    private final double LIFT_MAX = 0; //TODO: Test for Max Encoder Limit on Lift
    private final double LIFT_MIN = 0; //TODO: Test for Min Encoder Limit on Lift
    private static final double SERVO_LOCK_ONE = 0.0; // Needs to be tested;
    private static final double SERVO_UNLOCK_ONE = 0.3; // Needs to be tested;
    private static final double SERVO_LOCK_TWO = 0.3;
    private static final double SERVO_UNLOCK_TWO = 0.0;
    private boolean changeX = false;
    private boolean changeY = false;
    private boolean changeX2 = false;
    private boolean changeY2 = false;
    private boolean changeA = false;
    private boolean changeB = false;
    private boolean changeA2 = false;
    private boolean changeB2 = false;
    private boolean changeDpadLeft2 = false;
    private boolean teleArmToggle = false;
    private boolean telePincherToggle = false;
    private boolean teleFangToggle = false;


    public Stacker(LinearOpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.addLine("Stacker Init Started");
        opMode.telemetry.update();
        il = this.opMode.hardwareMap.dcMotor.get("il");
        ir = this.opMode.hardwareMap.dcMotor.get("ir");
        ll = this.opMode.hardwareMap.dcMotor.get("ll");
        lr = this.opMode.hardwareMap.dcMotor.get("lr");
        lFang = this.opMode.hardwareMap.servo.get("lFang");
        rFang = this.opMode.hardwareMap.servo.get("rFang");
        armRotater = this.opMode.hardwareMap.servo.get("armRotater");
        pincher = this.opMode.hardwareMap.servo.get("pincher");
        //gl = this.opMode.hardwareMap.servo.get("gl");
        //gr = this.opMode.hardwareMap.servo.get("gr");

        ll.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);
        //gl.setDirection(DcMotorSimple.Direction.FORWARD);
        //gr.setDirection(DcMotorSimple.Direction.REVERSE);

        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public CRServo getGr() {
        return gr;
    }

    public void setGr(CRServo gr) {
        this.gr = gr;
    }

    public CRServo getGl() {
        return gl;
    }

    public void setGl(CRServo gl) {
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
    public void setGantryPower(double power){
        gl.setPower(power);
        gr.setPower(power);
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
    public void setFangs (boolean lock) {
        if(lock){
            lFang.setPosition(SERVO_LOCK_TWO);
            rFang.setPosition(SERVO_LOCK_ONE);
        }
        else {
            lFang.setPosition(SERVO_UNLOCK_TWO);
            rFang.setPosition(SERVO_UNLOCK_ONE);
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
        if ((opMode_iterative.gamepad1.b && !changeB) || (opMode_iterative.gamepad2.a && !changeA2) && !telePincherToggle){
            setPincherPosition(true);
            telePincherToggle = !telePincherToggle;
        }
        else if ((opMode_iterative.gamepad1.b && !changeB) || (opMode_iterative.gamepad2.a && !changeA2) && telePincherToggle){
            setPincherPosition(false);
            telePincherToggle = !telePincherToggle;
        }
        changeB = opMode_iterative.gamepad1.b;
        changeA2 = opMode_iterative.gamepad2.a;
    }
    public void teleArm(){
        if (((opMode_iterative.gamepad1.b && !changeB ) || (opMode_iterative.gamepad2.b && !changeB2)) &&  !teleArmToggle){
            setArmPosition(true);
            teleArmToggle = !teleArmToggle;
        }
        else if ((opMode_iterative.gamepad1.b && !changeB ) || (opMode_iterative.gamepad2.b && !changeB2) && teleArmToggle){
            setArmPosition(false);
            teleArmToggle = !teleArmToggle;
        }
        changeB = opMode_iterative.gamepad1.b;
        changeB2 = opMode_iterative.gamepad2.b;
    }
    public void fangControl(){
        if((opMode_iterative.gamepad1.x && !changeX) || (opMode_iterative.gamepad2.x && !changeX2) &&  !teleFangToggle){
            setFangs(true);
            teleFangToggle = !teleFangToggle;
        }
        else if ((opMode_iterative.gamepad1.x && !changeX) || (opMode_iterative.gamepad2.x && !changeX2) &&  teleFangToggle){
            setFangs(false);
            teleFangToggle = !teleFangToggle;
        }
        changeX = opMode_iterative.gamepad1.x;
        changeX2 = opMode_iterative.gamepad2.x;

    }
    public void gantryControler(double power) {
        if (opMode_iterative.gamepad2.right_bumper){
            setGantryPower(power);
        }
        else if (opMode_iterative.gamepad2.left_bumper){
            setGantryPower(-power);
        }
        else if (opMode_iterative.gamepad1.y){
            setGantryPower(power);
        }
        else if (opMode_iterative.gamepad1.a){
            setGantryPower(-power);
        }
    }
    private void liftControl(double power) {
        if (opMode_iterative.gamepad2.right_trigger != 0){
            setLiftPower(opMode_iterative.gamepad2.right_trigger);
        }
        else if (opMode_iterative.gamepad2.left_trigger != 0){
            setLiftPower(-opMode_iterative.gamepad2.left_trigger);
        }
        else if (opMode_iterative.gamepad1.left_bumper){
            setLiftPower(-power);
        }
        else if (opMode_iterative.gamepad1.right_bumper) {
            setLiftPower(power);
        }
        setLiftPower(0);
    }
    private void intakeControl(double power){
        if (opMode_iterative.gamepad2.left_stick_y > 0) {
            setIntakePower(-power);
        }
        else if (opMode_iterative.gamepad2.left_stick_y < 0){
            setIntakePower(power);
        }
        else if(opMode_iterative.gamepad1.left_trigger > 0){
            setIntakePower(-power);
        }
        else if (opMode_iterative.gamepad1.right_trigger > 0){
            setIntakePower(power);
        }
    }
    public void stackerTeleControl(double intakePower, double liftPower, double gantryPower){
        teleArm();
        telePincher();
        fangControl();
        intakeControl(intakePower);
        liftControl(liftPower);
        gantryControler(gantryPower);

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

