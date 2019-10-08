package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Map;
import java.util.TreeMap;

// edit
public class DangerNoodle implements Robot {
    // Instance Variables
    private Drivetrain drivetrain;
    private Stacker manipulator;
    private BitmapVision bmv;
    private LinearOpMode opMode;

    private boolean isMoving;
    public ElapsedTime timer;
    private Map<String, Double> sensorVals;

    private HardwareThread hardwareThread;



    private Servo lFang;
    private Servo rFang;
    private static final double SERVO_LOCK = 0.0; // Needs to be tested;
    private static final double SERVO_UNLOCK = 0.0; // Needs to be tested;
    /*
        Constructor includes
     */
    // TODO: Add Exception Handling/Logging using RobotLog.i();
    public DangerNoodle(LinearOpMode opMode){
        try {
            timer = new ElapsedTime();
            this.opMode = opMode;
            this.sensorVals = new TreeMap<String, Double>();
            drivetrain = new Drivetrain(opMode, timer, sensorVals);
            manipulator = new Stacker(opMode);
            bmv = new BitmapVision(opMode);

            hardwareThread = new HardwareThread(this, sensorVals);
            hardwareThread.run();

            lFang = this.opMode.hardwareMap.servo.get("lFang");
            rFang = this.opMode.hardwareMap.servo.get("rFang");

        } catch (InterruptedException e) {
            // Include handling later.
            e.printStackTrace();
        }
        isMoving = false;
    }
    // Wait for robot
    @Override
    public void scan() {
        while(!opMode.opModeIsActive()){

        }

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

    public BitmapVision getBmv() {
        return bmv;
    }

    public void setBmv(BitmapVision bmv) {
        this.bmv = bmv;
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
}
