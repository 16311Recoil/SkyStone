package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;

// edit
public class DangerNoodle implements Robot {
    // Instance Variables
    private Drivetrain drivetrain;
    private Stacker manipulator;
    private BitmapVision bmv;
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private int[] skyPos;

    private boolean isMoving;
    public ElapsedTime timer;
    private Map<String, Double> sensorVals;

    private HardwareThread hardwareThread;



    /*
        Constructor includes
     */
    // TODO: Add Exception Handling/Logging using RobotLog.i();
    // TODO: Determine Hardware Thread Bug
    public DangerNoodle(LinearOpMode opMode){
        try {
            skyPos = new int[2];
            timer = new ElapsedTime();
            this.opMode = opMode;
            this.sensorVals = new ConcurrentHashMap<String, Double>();
            drivetrain = new Drivetrain(opMode, timer, sensorVals);
            manipulator = new Stacker(opMode);
            bmv = new BitmapVision(opMode);

            //hardwareThread = new HardwareThread(this, sensorVals);
            //hardwareThread.run();

        } catch (InterruptedException e) {
            // Include handling later.
            e.printStackTrace();
            RobotLog.i(e.getMessage());
            this.opMode.telemetry.addLine("DRIVETRAIN INIT FAILED");
            this.opMode.telemetry.update();
        }
        isMoving = false;
        opMode.telemetry.addLine("DangerNoodle Init Completed");
        opMode.telemetry.update();
    }
    public DangerNoodle(OpMode opMode){
        try {
            timer = new ElapsedTime();
            this.opMode_iterative = opMode;
            this.sensorVals = new ConcurrentHashMap<String, Double>();
            drivetrain = new Drivetrain(opMode_iterative, timer, sensorVals);
            manipulator = new Stacker(opMode_iterative);
            //bmv = new BitmapVision(opMode);


        } catch (InterruptedException e) {
            // Include handling later.
            e.printStackTrace();
            RobotLog.i(e.getMessage());
            this.opMode_iterative.telemetry.addLine("DRIVETRAIN INIT FAILED");
            this.opMode_iterative.telemetry.update();
        }
        isMoving = false;
        opMode_iterative.telemetry.addLine("DangerNoodle Init Completed");
        opMode_iterative.telemetry.update();
    }
    // Wait for robot
    @Override
    public void scan() {
        while(!opMode.isStarted() && skyPos[1] == 0){
            skyPos = bmv.getSkyPos();
        }
        if (skyPos[0] == 0)
            opMode.telemetry.addLine("LEFT");
        else if (skyPos[1] == 1)
            opMode.telemetry.addLine("MIDDLE");
        else
            opMode.telemetry.addLine("RIGHT");

    }


    @Override
    public void retrieval(double distance) {
        manipulator.setIntakePower(1);
        drivetrain.moveForward(distance, 0.8,3);
        manipulator.setIntakePower(0);

    }

    @Override
    public void moveFoundation(boolean direction) {
        if (direction) {
            /*
            // Latches onto foundation
            lFang.setPosition(SERVO_LOCK);
            rFang.setPosition(SERVO_LOCK);

            // Moves to zone (needs to be calculated)
            drivetrain.moveForward(1.5, 1, 5);

            // Unlocks
            lFang.setPosition(SERVO_UNLOCK);
            rFang.setPosition(SERVO_UNLOCK);
        } else {

            lFang.setPosition(SERVO_LOCK);
            rFang.setPosition(SERVO_LOCK);
            drivetrain.moveForward(1.5, 1, 5);
            lFang.setPosition(SERVO_UNLOCK);
            rFang.setPosition(SERVO_UNLOCK);
            */
        }


    }

    @Override
    public void navigate(boolean inside) {
        // while (getCurrentEncoderAverage < targetDist && opModeIsActive())
        drivetrain.moveForward(1,-1,3);
        // fix later
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drivetrain.move(1, 0,0,0,0);
        if (inside)
            drivetrain.move(0.85, 0, Math.PI/4,0,0);
         else
             drivetrain.move(1, 0, 0 ,0,0);


    }

    @Override
    public void placeSkystone() {

    }

    // TODO: Edit Drivetrain class to accomodate dynamic moinitoring of drivetrain.
    public double getVelocityEncoder(){
        // return drivetrain.sensorsVal.get("CurrentEncoderVal") - drivetrain.sensorsVal.get("Previous Encoder Val");
        return 0.0;
    }
    public void bulkRead(){

    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Stacker getManipulator() {
        return manipulator;
    }

    public void setManipulator(Stacker manipulator) {
        this.manipulator = manipulator;
    }
/*
    public BitmapVision getBmv() {
        return bmv;
    }
    */
/*
    public void setBmv(BitmapVision bmv) {
        this.bmv = bmv;
    }
    */

    public LinearOpMode getOpMode() {
        return opMode;
    }

    public void setOpMode(LinearOpMode opMode) {
        this.opMode = opMode;
    }
    public Map<String, Double> getSensorVals() {
        return sensorVals;
    }

    public void setSensorVals(Map<String, Double> sensorVals) {
        this.sensorVals = sensorVals;
    }
    public void teleOp(){
        drivetrain.moveTelop2(opMode_iterative.gamepad1.left_stick_x, opMode_iterative.gamepad1.left_stick_y, opMode_iterative.gamepad1.right_stick_x);
        manipulator.stackerTeleControl(0.75,1,.75);
    }

}
