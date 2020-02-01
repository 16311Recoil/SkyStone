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

import java.util.FormatFlagsConversionMismatchException;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentHashMap;

// edit
public class DangerNoodle implements Robot {
    // Instance Variables
    private Drivetrain drivetrain;
    private Stacker manipulator;
    //private BitmapVision bmv;
    private LinearOpMode opMode;
    private OpMode opMode_iterative;
    private int[] skyPos;

    private boolean isMoving;
    public ElapsedTime timer;
    private ConcurrentHashMap<String, Double> sensorVals;


    private HardwareThread hardwareThread;
    private ElapsedTime masterTime;
    private double LEFT, RIGHT, FORWARD, BACKWARD;
    private boolean blue;
    private BitmapVision bmv;


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
            hardwareThread = new HardwareThread(this, this.sensorVals);


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
            opMode.telemetry.addLine("RIGHT");
        else if (skyPos[1] == 1)
            opMode.telemetry.addLine("CENTER");
        else
            opMode.telemetry.addLine("LEFT");

    }



    @Override
    public void retrieval(double distance) {
        manipulator.setIntakePower(1);
        drivetrain.moveForward(distance, 0.8,3);
        manipulator.setIntakePower(0);

    }

    public void testCorrectTo() throws InterruptedException {

        double heading = getDrivetrain().getSensors().getFirstAngle();

        drivetrain.turnPID(78, 0.4 / 78, 0.014 / 78, 0, 4, false);

        Thread.sleep(2000);

        drivetrain.correctTo(0.3, heading, 5);

        Thread.sleep(2000);

        opMode.telemetry.addLine("1st done");
        opMode.telemetry.update();

        drivetrain.turnPID(78, 0.4 / 78, 0.014 / 78, 0, 4, true);

        Thread.sleep(2000);

        drivetrain.correctTo(0.3, heading, 5);

        opMode.telemetry.addLine("1st done");
        opMode.telemetry.update();


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

            drivetrain.move(0.25,0, BACKWARD,1170,5,0.05);

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

                drivetrain.move(0.4, 0, LEFT, 52, 3.675, 0.05);

                Thread.sleep(1000);

                drivetrain.move(.6, 0, FORWARD, 1500, 6, 0.05);
            }
            else {
                drivetrain.move(0.35, 0, RIGHT, 1300, 6, 0.1);

                Thread.sleep(400);

                drivetrain.move(0.35, 0, LEFT, 2, 3, 0.1);

                Thread.sleep(300);

                drivetrain.move(0.6, 0, FORWARD, 1500, 6, 0.05);
            }


            
        } else {
            drivetrain.move(0.6,0, BACKWARD,200,2,0.1);

            Thread.sleep(400);

            drivetrain.move(0.4,0, LEFT,1520,2,0.05);

            Thread.sleep(400);

            //drivetrain.correctHeading(0.3, heading, 4);

            //Thread.sleep(400);

            drivetrain.move(0.25,0, BACKWARD,1110,5,0.05);

            Thread.sleep(300);

            //drivetrain.correctHeading(3);


            manipulator.setFangs(true);

            Thread.sleep(1000);

            drivetrain.move(1,0, FORWARD, 1460,6,0.1);

            Thread.sleep(600);

            drivetrain.turnPID(90,(0.85 / 88),0,(0.2 / 88),4,true);

            Thread.sleep(400);

            manipulator.setFangs(true); //checking

            Thread.sleep(300);

            drivetrain.move(0.9,0, BACKWARD, 2000,6, 0.1);

            Thread.sleep(500);

            manipulator.setFangs(false);

            Thread.sleep(1000);

            if (skybridge){
                drivetrain.move(0.35, 0, LEFT, 1300, 4, 0.05);

                Thread.sleep(300);

                drivetrain.move(0.375, 0, RIGHT, 450, 3.65, 0.00);

                Thread.sleep(300);

                drivetrain.move(.6, 0, FORWARD, 1550, 6, 0.05);
            }
            else {
                drivetrain.move(0.35, 0, LEFT, 1300, 6, 0.1);

                Thread.sleep(400);

                drivetrain.move(0.25, 0, RIGHT, 5, 3, 0.1);

                Thread.sleep(300);

                drivetrain.move(0.6, 0, FORWARD, 1550, 6, 0.05);
            }
        }
    }

    public void skystone(boolean blue, boolean skybridge) throws InterruptedException{

        double heading = getDrivetrain().getSensors().getFirstAngle();
        opMode.telemetry.addData("InitAngle", heading);

        if (blue){
            switch (skyPos[0]) {
                case 2:
                    drivetrain.move(0.4,0, FORWARD,1030,2,0.1);

                    Thread.sleep(400);

                    drivetrain.turnPID(20, (0.276 / 20),(0.015 / 20), 0, 3, false);

                    Thread.sleep(1000);

                    drivetrain.move(0.4,0, FORWARD,200,2,0.1);

                    Thread.sleep(400);

                    manipulator.setIntakePower(-0.6);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, FORWARD,720,2,0.1);

                    Thread.sleep(2200);

                    //drivetrain.turnPID(20, (0.175 / 20),(0.0001 / 20), 0, 3, true);

                    manipulator.setIntakePower(0);


                    drivetrain.correctTo(0.245, heading,4);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, BACKWARD,830,4,0.1);

                    Thread.sleep(400);

                    drivetrain.correctTo(0.25, heading,2);

                    drivetrain.move(0.5,0, LEFT,82,5,0.1);

                    Thread.sleep(400);

                    manipulator.setIntakePower(0.4);

                    Thread.sleep(1500);

                    drivetrain.move(0.25,0, FORWARD,25,2,0.1);

                    drivetrain.correctTo(0.26, heading,2);

                    drivetrain.move(0.5,0, RIGHT,41,5,0.1);

                    /*manipulator.setLiftPosition(0.4, -300);

                    Thread.sleep(400);

                    manipulator.setGantryPosition(1, false);

                    Thread.sleep(2700);

                    drivetrain.move(0.35,0, BACKWARD, 800, 4, 0.1);

                    Thread.sleep(400);

                    manipulator.setLiftPosition(0.8,200);

                    Thread.sleep(400);

                    manipulator.setPincherPosition(true);

                    Thread.sleep(400);

                    manipulator.setLiftPosition(0.4,-300);

                    Thread.sleep(400);

                    manipulator.setArmPosition(0);

                    Thread.sleep(400);

                    manipulator.setArmPosition(0);

                    Thread.sleep(400);

                    manipulator.setGantryPosition(1,true);

                    Thread.sleep(2700);

                    drivetrain.move(0.6,0, FORWARD,200,2,0.1);*/

                    break;

                case 1:
                    drivetrain.move(0.4,0, FORWARD,450,2,0.1);

                    Thread.sleep(400);

                    drivetrain.move(0.30,0, LEFT,224,2.275,0.00);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, FORWARD,515,2,0.05);

                    Thread.sleep(400);

                    drivetrain.turnPID(18, (0.250 / 18),(0.015 / 18), 0, 2, true);

                    Thread.sleep(400);

                    manipulator.setIntakePower(-0.6);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, FORWARD,645,2,0.1);

                    Thread.sleep(2200);

                    manipulator.setIntakePower(0);

                    drivetrain.correctTo(0.245, heading,4);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, BACKWARD,665,3,0.1);

                    Thread.sleep(400);

                    drivetrain.correctTo(0.245, heading,4);

                    Thread.sleep(400);

                    drivetrain.move(0.4,0, LEFT,147,4,0.1);

                    Thread.sleep(400);

                    manipulator.setIntakePower(0.4);

                    Thread.sleep(1500);

                    drivetrain.correctTo(0.245, heading,4);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, RIGHT,47,3,0.1);

                    break;

                case 0:

                    drivetrain.move(0.35,0, FORWARD,940,2,0.1);

                    Thread.sleep(400);

                    drivetrain.turnPID(4, (0.07 / 4),(0.00005 / 4), 0, 2, true);

                    Thread.sleep(400);

                    manipulator.setIntakePower(-0.6);

                    Thread.sleep(400);

                    drivetrain.move(0.3,0, FORWARD,700,2,0.1);

                    Thread.sleep(1750);

                    manipulator.setIntakePower(0.5);

                    Thread.sleep(1000);

                    break;
            }
        }

        else {
            switch (skyPos[0]) {
                case 2:
                    drivetrain.move(0.4,0, FORWARD,450,2,0.1);

                    Thread.sleep(400);

                    drivetrain.move(0.35,0, RIGHT,265,2,0.00);

                    Thread.sleep(400);

                    drivetrain.move(0.4,0, FORWARD,640,2,0.05);

                    Thread.sleep(400);

                    drivetrain.turnPID(20, (0.3 / 20),(0.0 / 20), 0, 5, false);

                    Thread.sleep(1000);

                    manipulator.setIntakePower(-0.6);

                    Thread.sleep(400);

                    drivetrain.move(0.25,0, FORWARD,645,2,0.1);

                    Thread.sleep(3000);
                /*
                    drivetrain.move(0.35,0, BACKWARD, 800, 4, 0.1);

                    Thread.sleep(400);

                    manipulator.setLiftPosition(0.8,200);

                    Thread.sleep(400);

                    manipulator.setPincherPosition(true);

                    Thread.sleep(400);

                    manipulator.setLiftPosition(0.4,-300);

                    Thread.sleep(400);

                    manipulator.setArmPosition(0);

                    Thread.sleep(400);

                    manipulator.setArmPosition(0);

                    Thread.sleep(400);

                    manipulator.setGantryPosition(1,true);

                    Thread.sleep(2700);

                    drivetrain.move(0.6,0, FORWARD,200,2,0.1);*/

                    break;

                case 1:
                    break;

                case 0:
                    break;

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
        manipulator.stackerTeleControl(0.6, 0.5,1,1);
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

    public void setSensorVals(ConcurrentHashMap<String, Double> sensorVals) {
        this.sensorVals = sensorVals;
    }
    public void teleOp(){
        drivetrain.moveTelop2(opMode_iterative.gamepad1.left_stick_x, opMode_iterative.gamepad1.left_stick_y, opMode_iterative.gamepad1.right_stick_x);
        manipulator.stackerTeleControl(0.75,1,1,.75);
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
