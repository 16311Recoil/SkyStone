package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

public class DangerNoodle implements Robot {

    private static final double SERVO_LOCK = 0.0; // Needs to be tested;
    private static final double SERVO_UNLOCK = 0.0; // Needs to be tested;
    // Instance Variables
    private Drivetrain drivetrain;
    private Stacker manipulator;
    private BitmapVision bmv;
    private LinearOpMode opMode;

    private boolean isMoving;
    private ElapsedTime timer;
    private double prevEncoder;

    private double change2Encoder;

    private Servo lFang;
    private Servo rFang;

    /*
        Constructor includes
     */
    // TODO: Add Exception Handling/Logging using RobotLog.i();
    public DangerNoodle(LinearOpMode opMode){
        try {
            this.opMode = opMode;
            drivetrain = new Drivetrain(opMode);
            manipulator = new Stacker();
            bmv = new BitmapVision(opMode);

            lFang = this.opMode.hardwareMap.servo.get("lFang");
            rFang = this.opMode.hardwareMap.servo.get("rFang");

        } catch (InterruptedException e) {
            // Include handling later.
            e.printStackTrace();
        }
        isMoving = false;
        timer = new ElapsedTime();
        prevEncoder = drivetrain.getEncoderAverage();
    }
    // Wait for robot
    @Override
    public void scan() {

    }

    @Override
    public void retrieval(double distance) {
        manipulator.intake(1);
        drivetrain.moveForward(distance, 0.8,3);
        manipulator.intake(0);

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
        drivetrain.move(1, 0,0);
        if (inside)
            drivetrain.move(0.85, 0, (Math.PI / 4));
         else
             drivetrain.move(1, 0, 0 );


    }

    @Override
    public void placeSkystone() {

    }
    /*
        monitorEncoders();
        - Monitors encoders during every bulk read.
        -  change in change in encoder ticks > 0.
        - Value of Encoder ticks != 0.
     */
    public void monitorEncoders(double encoderAverage){
        if (isMoving) {
            //double changeEncoder = drivetrain.sensorsVals.get("PrevEncoderAverage") - drivetrain.sensprVals.get("CurrentEnc");


        }


    }
    // TODO: Edit Drivetrain class to accomodate dynamic moinitoring of drivetrain.
    public double getVelocityEncoder(){
        // return drivetrain.sensorsVal.get("CurrentEncoderVal") - drivetrain.sensorsVal.get("Previous Encoder Val");
        return 0.0;
    }
}
