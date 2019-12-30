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

import static android.os.SystemClock.sleep;


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
    private final double TIME_FOR_GANTRY_IN = 0; //TODO: Test for Times
    private final double TIME_FOR_GANTRY_OUT = 0;
    private final double CLOSED_PINCHER_SERVO_POSITION = 0.75;
    private final double OPEN_PICHER_SERVO_POSITION = 0;
    private final double[] ARM_POSITIONS = new double[]{0, 0.55, 0.9};
    private int armSpot = 1;
    private int armRotation = -1;
    private int liftSpotCurrentMax = 1;
    private int liftEncoderVal = 0;
    private final double INCHES_TO_SERVO = 0;//TODO: Test conversions for inches in gantry movement to servo position
    private final double LIFT_MAX = -8500; //TODO: Test for Max Encoder Limit on Lift
    private final double[] LIFT_BLOCK = new double[]{600, -8500}; //TODO: Test for encoder readings at each block height
    private final double LIFT_MIN = 600; //TODO: Test for Min Encoder Limit on Lift
    private static final double SERVO_LOCK = 0; // Needs to be tested;
    private static final double SERVO_UNLOCK = 0.32; // Needs to be tested;

    private boolean changeX = false;
    private boolean changeY = false;
    private boolean changeX2 = false;
    private boolean changeY2 = false;
    private boolean changeA = false;
    private boolean changeB = false;
    private boolean changeA2 = false;
    private boolean changeB2 = false;
    private boolean changeDpadUp2 = false;
    private boolean changeDpadDown2 = false;
    private boolean changeDpadLeft2 = false;
    private boolean changeDpadRight2 = false;
    private boolean telePincherToggle = false;
    private boolean teleFangToggle = false;
    private boolean teleLiftMax = false;
    private boolean buttonA;
    private boolean buttonB;
    private boolean buttonX;
    private boolean buttonY;


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

        gl = this.opMode.hardwareMap.crservo.get("gl");
        gr = this.opMode.hardwareMap.crservo.get("gr");

        lFang.setDirection(Servo.Direction.FORWARD);
        rFang.setDirection(Servo.Direction.REVERSE);

        ll.setDirection(DcMotor.Direction.FORWARD);
        lr.setDirection(DcMotor.Direction.REVERSE);

        il.setDirection(DcMotor.Direction.FORWARD);
        ir.setDirection(DcMotor.Direction.REVERSE);

        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gl.setDirection(DcMotorSimple.Direction.FORWARD);
        gr.setDirection(DcMotorSimple.Direction.FORWARD);

        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ir.setPower(0);
        il.setPower(0);

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
        lFang = this.opMode_iterative.hardwareMap.servo.get("lFang");
        rFang = this.opMode_iterative.hardwareMap.servo.get("rFang");
        gl = this.opMode_iterative.hardwareMap.crservo.get("gl");
        gr = this.opMode_iterative.hardwareMap.crservo.get("gr");
        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ir.setPower(0);
        il.setPower(0);

        il.setDirection(DcMotor.Direction.FORWARD);
        ir.setDirection(DcMotor.Direction.REVERSE);

        gl.setDirection(CRServo.Direction.FORWARD);
        gr.setDirection(CRServo.Direction.REVERSE);

        lFang.setDirection(Servo.Direction.FORWARD);
        rFang.setDirection(Servo.Direction.REVERSE);

        ll.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.FORWARD);

        ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        gl.setPower(-power);
        gr.setPower(-power);
    }

    public void setArmPosition(int position){
        armRotater.setPosition(ARM_POSITIONS[position]);
    }

    public void setPincherPosition(boolean close) {
        if (close) {
            pincher.setPosition(CLOSED_PINCHER_SERVO_POSITION);
        } else {
            pincher.setPosition(OPEN_PICHER_SERVO_POSITION);
        }
    }

    public void setFangs (boolean lock) {
        if(lock){
            lFang.setPosition(SERVO_LOCK);
            rFang.setPosition(SERVO_LOCK);
        }
        else {
            lFang.setPosition(SERVO_UNLOCK);
            rFang.setPosition(SERVO_UNLOCK);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void intakeTime(double power) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < TIME_FOR_INTAKE) {
            setIntakePower(power);
        }
        setIntakePower(0);
    }

    public void setArmPositionToggle() {
        if (armSpot == 0 || armSpot == 2){
            armRotation *= -1;
        }
        armSpot += 1 * armRotation;
        setArmPosition(armSpot);
    }

    public void setLiftPosition(double power, double position) {
        double currentPos = getLiftEncoderAverage();
        if (currentPos < position) {
            while (currentPos < position) {
                setLiftPower(power);
            }
        }
        else {
            while (currentPos > position) {
                setLiftPower(-power);
            }
        }
        setLiftPower(0);
    }

    public void setGantryPosition(double power, boolean in){
        ElapsedTime timer = new ElapsedTime();
        if (in) {
            while (timer.milliseconds() < TIME_FOR_GANTRY_IN) {
                setGantryPower(power);
            }
        }
        else {
            while (timer.milliseconds() < TIME_FOR_GANTRY_OUT) {
                setGantryPower(-power);
            }
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
        if (((opMode_iterative.gamepad1.b && !changeB ) || (opMode_iterative.gamepad2.b && !changeB2))){
            setArmPositionToggle();
        }
        changeB = opMode_iterative.gamepad1.b;
        changeB2 = opMode_iterative.gamepad2.b;
    }
    public void fangControl(){
        if(((opMode_iterative.gamepad1.x && !changeX) || (opMode_iterative.gamepad2.x && !changeX2)) &&  !teleFangToggle){
            setFangs(false);

            teleFangToggle = !teleFangToggle;
            opMode_iterative.telemetry.addLine("FANGS LOCKED");
            opMode_iterative.telemetry.update();
        }
        else if (((opMode_iterative.gamepad1.x && !changeX2) || (opMode_iterative.gamepad2.y && !changeY2)) &&  teleFangToggle){
            setFangs(true);
            teleFangToggle = !teleFangToggle;
            opMode_iterative.telemetry.addLine("FANGS UNLOCKED");
            opMode_iterative.telemetry.update();
        }
        changeX = opMode_iterative.gamepad1.x;
        changeX2 = opMode_iterative.gamepad2.x;
        changeY2 = opMode_iterative.gamepad2.y;

    }
    public void gantryController(double power) {
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
        else {
            setGantryPower(0);
        }
    }
    private void liftControl(double power) {  //TODO: Update with encoder stops, use LIFT_BLOCK[liftMaxSpot] when doing so
        if (opMode_iterative.gamepad2.right_trigger != 0){
            setLiftPower(opMode_iterative.gamepad2.right_trigger);
        }
        else if (opMode_iterative.gamepad2.left_trigger != 0) {
            setLiftPower(-opMode_iterative.gamepad2.left_trigger * 0.5);
        }
        else if (opMode_iterative.gamepad1.left_bumper){
            setLiftPower(-power * 0.5);
        }
        else if (opMode_iterative.gamepad1.right_bumper) {
            setLiftPower(power);
        }
        else{
            setLiftPower(0);
        }
    }

    /*private void liftControl(double power) {  //TODO: Update with encoder stops, use LIFT_BLOCK[liftMaxSpot] when doing so
        if (opMode_iterative.gamepad2.right_trigger != 0){
            liftEncoderVal += opMode_iterative.gamepad2.right_trigger * 10;
        }
        else if (opMode_iterative.gamepad2.left_trigger != 0) {
            liftEncoderVal -= opMode_iterative.gamepad2.left_trigger * 10;
        }
        else if (opMode_iterative.gamepad1.left_bumper){
            liftEncoderVal -= Math.round(power) * 10;
        }
        else if (opMode_iterative.gamepad1.right_bumper) {
            liftEncoderVal += Math.round(power) * 10;
        }
        setLiftPosition(power, liftEncoderVal);
    }*/

    /*private void liftControl(double power) {  //TODO: Update with encoder stops, use LIFT_BLOCK[liftMaxSpot] when doing so
        if ((opMode_iterative.gamepad2.right_trigger != 0) && (getLiftEncoderAverage() > LIFT_BLOCK[liftMaxSpot])){
            setLiftPower(opMode_iterative.gamepad2.right_trigger);
        }
        else if ((opMode_iterative.gamepad2.left_trigger != 0) && (getLiftEncoderAverage() < LIFT_BLOCK[0])){
            setLiftPower(-opMode_iterative.gamepad2.left_trigger);
        }
        else if (opMode_iterative.gamepad1.left_bumper && (getLiftEncoderAverage() < LIFT_BLOCK[0])){
            setLiftPower(-power);
        }
        else if (opMode_iterative.gamepad1.right_bumper && (getLiftEncoderAverage() > LIFT_BLOCK[liftMaxSpot])) {
            setLiftPower(power);
        }
        else{
            setLiftPower(0);
        }
    }*/

    private void liftMaxControl(){
        if (opMode_iterative.gamepad2.dpad_up && !changeDpadUp2 && (liftSpotCurrentMax < LIFT_BLOCK.length)){ //TODO Update this number for the max size of LIFT_BLOCK
            liftSpotCurrentMax++;
        }
        if (opMode_iterative.gamepad2.dpad_down && !changeDpadDown2 && liftSpotCurrentMax > 0){
            liftSpotCurrentMax--;
        }
        changeDpadUp2 = opMode_iterative.gamepad2.dpad_up;
        changeDpadDown2 = opMode_iterative.gamepad2.dpad_down;
    }
    private void macControl(){
        if (opMode_iterative.gamepad2.dpad_left && !changeDpadLeft2){
            macIn();
        }
        if (opMode_iterative.gamepad2.dpad_right && !changeDpadRight2){
            macOut();
        }
    }

    private void intakeControl(double power){
        if (opMode_iterative.gamepad2.left_stick_y > 0) {
            setIntakePower(power);
        }
        else if (opMode_iterative.gamepad2.left_stick_y < 0){
            setIntakePower(-power);
        }
        else if(opMode_iterative.gamepad1.left_trigger > 0){
            setIntakePower(power);
        }
        else if (opMode_iterative.gamepad1.right_trigger > 0){
            setIntakePower(-power);
        }
        else{
            setIntakePower(0);
        }

    }
    public void stackerTeleControl(double intakePower, double liftPower, double gantryPower){
        teleArm();
        telePincher();
        fangControl();
        intakeControl(intakePower);
        liftControl(liftPower);
        gantryController(gantryPower);
    }
    public void buttonMapping(){
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
////////////////////////////////////// Auto Macros //////////////////////////////////////////////////////////////////////////
    public void macOut(){
        setPincherPosition(true);
        sleep(500);
        setLiftPosition(1, LIFT_BLOCK[0]);
        sleep(500);
        setGantryPosition(1, false);
        sleep(500);
        setArmPosition(0);
    }
    public void macIn(){
        setPincherPosition(true);
        setArmPosition(2);
        sleep(500);
        setGantryPosition(1,true);
        sleep(500);
        setLiftPosition(1, LIFT_MIN);
    }
}

