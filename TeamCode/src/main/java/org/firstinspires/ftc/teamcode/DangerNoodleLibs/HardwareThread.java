package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.jetbrains.annotations.*;

import java.util.Map;

public class HardwareThread implements Runnable {

    private Thread thread;

    private DangerNoodle robot;
    private Sensors sensors;
    private Stacker manip;
    public Map<String, Double> sensorVals;

    private LinearOpMode opMode;

    private double[] drivetrainEncoders;
    private double[] liftEncoders;

    private final ExpansionHubEx expansionHubEx, expansionHubEx2;

    private Drivetrain drivetrain;
    private RevBulkData bulkData, bulkData2;



    public HardwareThread(@NotNull DangerNoodle robot, Map<String, Double> sensorVals){
        drivetrainEncoders = new double[4];
        liftEncoders = new double[2];

        this.opMode = robot.getOpMode();

        opMode.telemetry.addLine("Thread Init Started");
        opMode.telemetry.update();

        this.sensorVals = sensorVals;
        this.robot = robot;
        this.drivetrain = robot.getDrivetrain();
        this.sensors = robot.getDrivetrain().getSensors();
        this.manip = robot.getManipulator();


        expansionHubEx = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHubEx2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        bulkData = expansionHubEx.getBulkInputData();
        bulkData2 = expansionHubEx2.getBulkInputData();

        start();

        drivetrain.resetEncoders();

        drivetrainEncoders[0] = bulkData.getMotorCurrentPosition(drivetrain.getFl());
        drivetrainEncoders[1] = bulkData2.getMotorCurrentPosition(drivetrain.getFr());
        drivetrainEncoders[2] = bulkData.getMotorCurrentPosition(drivetrain.getBl());
        drivetrainEncoders[3] = bulkData2.getMotorCurrentPosition(drivetrain.getBr());

        opMode.telemetry.addLine("3");
        opMode.telemetry.update();

        // initialize lift encoders
        liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getIl());
        liftEncoders[1] = bulkData2.getMotorCurrentPosition(manip.getIr());

        // get sensors
        //      - gyro
        //      - REV 2m Distance Sensor
        sensorVals.put("Previous Drivetrain Encoder Average", drivetrain.getEncoderAverage(Math.PI/2));
        sensorVals.put("Previous Lift Encoder Average", robot.getManipulator().getLiftEncoderAverage());
        sensorVals.put("Previous Time", robot.timer.milliseconds());
        sensorVals.put("Init Gyro Angle", sensors.getFirstAngle());

        sensorVals.put("X", sensors.getXDistance());
        sensorVals.put("Y", sensors.getYDistance());

        opMode.telemetry.addLine("Thread Init Completed");
        opMode.telemetry.update();

    }
    public void start(){
        if (thread == null){
            thread = new Thread (this, "Hardware Thread");
            thread.start ();
        }
    }

    @Override
    public void run(){
        bulkData = expansionHubEx.getBulkInputData();
        bulkData2 = expansionHubEx2.getBulkInputData();

        sensorVals.put("Current Drivetrain Encoder Average", drivetrain.getEncoderAverage(Math.PI/2));
        sensorVals.put("Current Lift Encoder Average", manip.getLiftEncoderAverage());
        sensorVals.put("Current Time", robot.timer.milliseconds());

        //caching?
        drivetrainEncoders[0] = bulkData.getMotorCurrentPosition(drivetrain.getFl());
        drivetrainEncoders[1] = bulkData2.getMotorCurrentPosition(drivetrain.getFr());
        drivetrainEncoders[2] = bulkData.getMotorCurrentPosition(drivetrain.getBl());
        drivetrainEncoders[3] = bulkData2.getMotorCurrentPosition(drivetrain.getBr());

        liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getIl());
        liftEncoders[1] = bulkData2.getMotorCurrentPosition(manip.getIr());
        // update rev 2m distance


        sensorVals.put("Previous Drivetrain Encoder Average", drivetrain.getEncoderAverage(Math.PI/2));
        sensorVals.put("Previous Lift Encoder Average", robot.getManipulator().getLiftEncoderAverage());
        sensorVals.put("Previous Time", robot.timer.milliseconds());

        sensorVals.put("X", sensors.getXDistance());
        sensorVals.put("Y", sensors.getYDistance());
    }
}
