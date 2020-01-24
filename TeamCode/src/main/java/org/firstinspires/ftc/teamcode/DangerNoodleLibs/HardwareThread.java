package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.jetbrains.annotations.*;

import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class HardwareThread implements Runnable {
    private Thread thread;

    private DangerNoodle robot;
    private Sensors sensors;
    private Stacker manip;
    public Map<String, Double> sensorVals;

    private LinearOpMode opMode;

    private int[] drivetrainEncoders;
    private double[] liftEncoders;

    private final ExpansionHubEx expansionHubEx, expansionHubEx2;

    private Drivetrain drivetrain;
    private RevBulkData bulkData, bulkData2;



    public HardwareThread(@NotNull DangerNoodle robot, ConcurrentHashMap<String, Double> sensorVals) throws NullPointerException{
        start();

        drivetrainEncoders = new int[4];
        liftEncoders = new double[2];

        this.opMode = robot.getOpMode();


        this.sensorVals = sensorVals;
        this.robot = robot;
        this.drivetrain = robot.getDrivetrain();
        this.sensors = robot.getDrivetrain().getSensors();
        this.manip = robot.getManipulator();


        expansionHubEx = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHubEx2 = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        bulkData = expansionHubEx.getBulkInputData();
        bulkData2 = expansionHubEx2.getBulkInputData();

        drivetrain.resetEncoders();

        drivetrainEncoders[0] = bulkData.getMotorCurrentPosition(drivetrain.getFr());
        drivetrainEncoders[1] = bulkData2.getMotorCurrentPosition(drivetrain.getFl());
        drivetrainEncoders[2] = bulkData.getMotorCurrentPosition(drivetrain.getBr());
        drivetrainEncoders[3] = bulkData2.getMotorCurrentPosition(drivetrain.getBl());


        // initialize lift encoders
        //liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getLl());
        //liftEncoders[1] = bulkData2.getMotorCurrentPosition(manip.getLr());

        // get sensors
        //      - gyro
        //      - REV 2m Distance Sensor
        sensorVals.put("X", sensors.getXDistance());
        sensorVals.put("Y", sensors.getYDistance());

        //sensorVals.put("Previous Lift Encoder Average", robot.getManipulator().getLiftEncoderAverage());
        sensorVals.put("Previous Time", robot.timer.milliseconds());
        sensorVals.put("Current Angle", sensors.getFirstAngle());
        sensorVals.put("Previous Drivetrain Encoder Average", drivetrain.getEncoderAverage(sensorVals.get("Current Angle")));



    }
    public void start(){
        if (thread == null){
            thread = new Thread (this, "Hardware Thread");
            thread.start();
        }
    }

    @Override
    public void run(){
        while(!opMode.isStopRequested()) {
            bulkData = expansionHubEx.getBulkInputData();
            bulkData2 = expansionHubEx2.getBulkInputData();
            double angle;
            try {
                angle = sensorVals.get("Current Angle");
            } catch (NullPointerException E) {
                opMode.telemetry.addLine("Gyro Angle Not Set - NullPtr");
                angle = Math.PI / 2;
            }

            opMode.telemetry.addLine("STUCK IN HARDWARE THREAD");
            opMode.telemetry.update();


            sensorVals.put("Current Drivetrain Encoder Average", getEncoderAverage());
            sensorVals.put("Current Lift Encoder Average", manip.getLiftEncoderAverage());
            sensorVals.put("Current Time", robot.timer.milliseconds());

            //caching?
            drivetrainEncoders[0] = Math.abs(bulkData.getMotorCurrentPosition(drivetrain.getFl()));
            drivetrainEncoders[1] = Math.abs(bulkData2.getMotorCurrentPosition(drivetrain.getFr()));
            drivetrainEncoders[2] = Math.abs(bulkData.getMotorCurrentPosition(drivetrain.getBl()));
            drivetrainEncoders[3] = Math.abs(bulkData2.getMotorCurrentPosition(drivetrain.getBr()));

            RobotLog.vv("BULK READ MESSAGE","BULK READING");
            RobotLog.i(Arrays.toString(drivetrainEncoders));


            drivetrain.setEncoderVals(drivetrainEncoders);
            sensorVals.put("FL", 1.0 * bulkData.getMotorCurrentPosition(drivetrain.getFl()));
            sensorVals.put("Current Drivetrain Encoder Average", drivetrain.getEncoderAverage(angle));

            sensorVals.put("X", sensors.getXDistance());
            sensorVals.put("Y", sensors.getYDistance());
            thread.yield();

            //liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getIl());
            //liftEncoders[1] = bulkData2.getMotorCurrentPosition(manip.getIr());
            // update rev 2m distance
        }

    }
    public double getEncoderAverage() {
        double encoderAverage = 0;
        int counter = 0;
        for (int i = 0; i < drivetrainEncoders.length; i++) {
            if (drivetrainEncoders[i] == 0) {
                counter++;
            }
            encoderAverage += drivetrainEncoders[i];
        }
        if (counter == 4)
            return 0;
        else {
            encoderAverage /= (4 - counter);
            return encoderAverage;
        }
    }
}
