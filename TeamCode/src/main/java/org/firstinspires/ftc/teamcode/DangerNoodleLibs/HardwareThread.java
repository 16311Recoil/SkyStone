package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodle.Robot;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;
import org.jetbrains.annotations.*;

import java.util.Map;

public class HardwareThread implements Runnable {

    private final ExpansionHubEx expansionHubEx2;
    private Drivetrain drivetrain;
    private RevBulkData bulkData;
    private ExpansionHubEx expansionHubEx;
    private DangerNoodle robot;
    public Map<String, Double> sensorVals;
    double[] drivetrainEncoders = new double[4];
    double[] liftEncoders = new double[2];
    private Sensors sensors;
    private Stacker manip;
    private RevBulkData bulkData2;

    public HardwareThread(@NotNull DangerNoodle robot, Map<String, Double> sensorVals){
        this.sensorVals = sensorVals;
        this.robot = robot;
        this.drivetrain = robot.getDrivetrain();
        this.sensors = robot.getDrivetrain().getSensors();
        this.manip = robot.getManipulator();
        expansionHubEx = robot.getOpMode().hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        expansionHubEx2 = robot.getOpMode().hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData = expansionHubEx.getBulkInputData();
        drivetrainEncoders[0] = bulkData.getMotorCurrentPosition(drivetrain.getFl());
        drivetrainEncoders[1] = bulkData.getMotorCurrentPosition(drivetrain.getFr());
        drivetrainEncoders[2] = bulkData.getMotorCurrentPosition(drivetrain.getBl());
        drivetrainEncoders[3] = bulkData.getMotorCurrentPosition(drivetrain.getBr());

        // initialize lift encoders
        liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getIl());
        liftEncoders[1] = bulkData.getMotorCurrentPosition(manip.getIr());

        // get sensors
        //      - gyro
        //      - REV 2m Distance Sensor
        sensorVals.put("Previous Drivetrain Encoder Average", drivetrain.getEncoderAverage(drivetrainEncoders));
        sensorVals.put("Previous Lift Encoder Average", robot.getManipulator().getLiftEncoderAverage());
        sensorVals.put("Previous Time", robot.timer.milliseconds());
        sensorVals.put("Init Gyro Angle", sensors.getFirstAngle());

    }

    @Override
    public void run() {
        while(!robot.getOpMode().isStopRequested()){
            // continuously update sensorVals and
            bulkData = expansionHubEx.getBulkInputData();
            bulkData2 = expansionHubEx2.getBulkInputData();
            sensorVals.put("Previous Drivetrain Encoder Average", drivetrain.getEncoderAverage(drivetrainEncoders));
            sensorVals.put("Previous Lift Encoder Average", robot.getManipulator().getLiftEncoderAverage());
            sensorVals.put("Previous Time", robot.timer.milliseconds());
            sensorVals.put("Init Gyro Angle", sensors.getFirstAngle());

            //caching?
            drivetrainEncoders[0] = bulkData.getMotorCurrentPosition(drivetrain.getFl());
            drivetrainEncoders[1] = bulkData.getMotorCurrentPosition(drivetrain.getFr());
            drivetrainEncoders[2] = bulkData.getMotorCurrentPosition(drivetrain.getBl());
            drivetrainEncoders[3] = bulkData.getMotorCurrentPosition(drivetrain.getBr());

            liftEncoders[0] = bulkData.getMotorCurrentPosition(manip.getIl());
            liftEncoders[1] = bulkData.getMotorCurrentPosition(manip.getIr());
            // update rev 2m distance


            sensorVals.put("Current Drivetrain Encoder Average", drivetrain.getEncoderAverage(drivetrainEncoders));
            sensorVals.put("Current Lift Encoder Average", manip.getLiftEncoderAverage());
            sensorVals.put("Current Time", robot.timer.milliseconds());
        }
    }

}
