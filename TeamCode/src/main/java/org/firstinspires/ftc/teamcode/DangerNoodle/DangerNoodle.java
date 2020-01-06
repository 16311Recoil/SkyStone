package org.firstinspires.ftc.teamcode.DangerNoodle;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;
import org.firstinspires.ftc.teamcode.R;
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
    private ElapsedTime masterTime;
    private double LEFT, RIGHT, FORWARD, BACKWARD;
    private boolean blue;



    /*
        Constructor includes
     */
    // TODO: Add Exception Handling/Logging using RobotLog.i();
    // TODO: Determine Hardware Thread Bug
    public DangerNoodle(LinearOpMode opMode, boolean blue, boolean foundation){
        try {
            skyPos = new int[2];
            timer = new ElapsedTime();
            this.opMode = opMode;
            this.sensorVals = new ConcurrentHashMap<String, Double>();
            drivetrain = new Drivetrain(opMode, timer, sensorVals);
            manipulator = new Stacker(opMode);
            bmv = new BitmapVision(opMode);
            masterTime = new ElapsedTime();

            //hardwareThread = new HardwareThread(this, sensorVals);
            //hardwareThread.run();

        } catch (InterruptedException e) {
            // Include handling later.
            e.printStackTrace();
            RobotLog.i(e.getMessage());
            this.opMode.telemetry.addLine("DRIVETRAIN INIT FAILED");
            this.opMode.telemetry.update();
        }
        this.blue = blue;
        if (blue && foundation) {
            LEFT = Math.PI;
            RIGHT = 0;
            FORWARD = Math.PI / 2;
            BACKWARD = 3* Math.PI / 2;
        } else if (blue){
            LEFT = Math.PI;
            RIGHT = 0;
            FORWARD = Math.PI / 2;
            BACKWARD = 3 * Math.PI / 2;
        } else if (foundation){
            LEFT = Math.PI;
            RIGHT = 0;
            FORWARD = Math.PI / 2;
            BACKWARD = 3 * Math.PI / 2;
        } else {
            LEFT = Math.PI;
            RIGHT = 0;
            FORWARD = Math.PI / 2;
            BACKWARD = 3 * Math.PI / 2;
        }

        isMoving = false;
        hardwareThread = new HardwareThread(this, sensorVals);
        opMode.telemetry.addLine("DangerNoodle Init Completed");
        opMode.telemetry.update();
    }
    public double getX(){return sensorVals.get("X");}
    public double getY(){return sensorVals.get("Y");}
    public DangerNoodle(OpMode opMode){
        try {
            timer = new ElapsedTime();
            this.opMode_iterative = opMode;
            this.sensorVals = new ConcurrentHashMap<String, Double>();
            drivetrain = new Drivetrain(opMode_iterative, timer, sensorVals);
            manipulator = new Stacker(opMode_iterative);
            //bmv = new BitmapVision(opMode);
            masterTime = new ElapsedTime();


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
    public void moveFoundation(boolean blue, boolean skybridge) throws InterruptedException {

        double heading = getDrivetrain().getSensors().getFirstAngle();
        opMode.telemetry.addData("InitAngle", heading);

        manipulator.setFangs(false);

        if (blue){

            drivetrain.move(0.6,0, BACKWARD,200,2,0.1);

            Thread.sleep(400);

            drivetrain.move(0.35,0, RIGHT,1300,2,0.1);

            Thread.sleep(400);

            //drivetrain.correctHeading(0.3, heading, 4);

            Thread.sleep(400);

            drivetrain.move(0.25,0, BACKWARD,1180,5,0.05);

            Thread.sleep(300);


            //drivetrain.correctHeading(3);

            Thread.sleep(400);

            manipulator.setFangs(true);

            Thread.sleep(1000);

            drivetrain.move(1,0, FORWARD, 1200,6,0.1);

            Thread.sleep(400);

            Thread.sleep(1000);

            drivetrain.turnPID(90,(0.85 / 88),0,(0.2 / 88),5,false);

            Thread.sleep(400);

            manipulator.setFangs(true); //checking

            Thread.sleep(400);

            drivetrain.move(0.9,0, BACKWARD, 2000,6, 0.1);

            Thread.sleep(500);

            manipulator.setFangs(false);

            Thread.sleep(1000);
            if (skybridge) {

                // Move to wall

                drivetrain.move(0.35, 0, RIGHT, 1300, 4, 0.05);

                Thread.sleep(300);

                drivetrain.move(0.35, 0, LEFT, 1200, 4, 0.05);

                Thread.sleep(1000);

                drivetrain.move(.6, 0, FORWARD, 1550, 6, 0.05);
            }
            else {
                drivetrain.move(0.35, 0, RIGHT, 1100, 6, 0.1);

                Thread.sleep(400);

                drivetrain.move(0.35, 0, LEFT, 50, 3, 0.1);

                Thread.sleep(300);

                drivetrain.move(0.6, 0, FORWARD, 1550, 6, 0.05);
            }


            
        } else {
            drivetrain.move(0.6,0, BACKWARD,200,2,0.1);

            Thread.sleep(400);

            drivetrain.move(0.35,0, LEFT,1250,2,0.1);

            Thread.sleep(400);

            //drivetrain.correctHeading(0.3, heading, 4);

            //Thread.sleep(400);

            drivetrain.move(0.25,0, BACKWARD,1180,5,0.05);

            Thread.sleep(300);

            //drivetrain.correctHeading(3);


            manipulator.setFangs(true);

            Thread.sleep(1000);

            drivetrain.move(1,0, FORWARD, 1200,6,0.1);

            Thread.sleep(400);

            drivetrain.turnPID(90,(0.85 / 88),0,(0.2 / 88),4,true);

            Thread.sleep(400);

            manipulator.setFangs(true); //checking

            Thread.sleep(300);

            drivetrain.move(0.9,0, BACKWARD, 2000,6, 0.1);

            Thread.sleep(500);

            manipulator.setFangs(false);

            Thread.sleep(1000);

            if (skybridge){
                drivetrain.move(0.35, 0, RIGHT, 360, 4, 0.05);

                Thread.sleep(300);

                drivetrain.move(.6, 0, FORWARD, 1550, 6, 0.05);
            }
            else {
                drivetrain.move(0.35, 0, LEFT, 1100, 6, 0.1);

                Thread.sleep(400);

                drivetrain.move(0.35, 0, RIGHT, 50, 3, 0.1);

                Thread.sleep(300);

                drivetrain.move(0.6, 0, FORWARD, 1550, 6, 0.05);
            }
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
        drivetrain.move(1, 0,0,0,0,0.1);
        if (inside)
            drivetrain.move(0.85, 0, Math.PI/4,0,0,0.1);
         else
             drivetrain.move(1, 0, 0 ,0,0,0.1);
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

    public void teleopControls(){
        drivetrain.moveTelop2(opMode_iterative.gamepad1.right_stick_x, -opMode_iterative.gamepad1.right_stick_y, opMode_iterative.gamepad1.left_stick_x);
        drivetrain.toggleSpeed();
        manipulator.stackerTeleControl(0.6,1,1);
    }

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
    public String toString(){
        String output = "DANGERNOODLE OUTPUT\n";
        output += "\tTIMER (ms): " + masterTime.milliseconds();
        output += "\n\tSENSOR VALS:";
        output += "\n\t\tDT Encoder Average: " + sensorVals.get("Current Drivetrain Encoder Average");
        output += "\n\t\tX: " + sensorVals.get("X") + "\tY:" + sensorVals.get("Y");
        return output;
    }


}
