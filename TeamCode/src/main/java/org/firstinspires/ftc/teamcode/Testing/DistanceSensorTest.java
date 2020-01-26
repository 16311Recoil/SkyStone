package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.RecursiveAction;
@TeleOp
        (name = "Sensors Test 2", group = "Controlled")
public class DistanceSensorTest extends LinearOpMode {

    HardwareThread thread;
    DangerNoodle noodle;
    ConcurrentHashMap<String, Double> sensorVals;
    private RevBulkData bulkData, bulkData2;
    @Override
    public void runOpMode() throws InterruptedException {
        final ExpansionHubEx expansionHubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        final ExpansionHubEx expansionHubEx2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        bulkData = expansionHubEx.getBulkInputData();
        bulkData2 = expansionHubEx2.getBulkInputData();


        sensorVals = new ConcurrentHashMap<>();
        noodle = new DangerNoodle(this);

        sensorVals.put("FL", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getFl()));
        sensorVals.put("Fr", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getFr()));
        sensorVals.put("Bl", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getBl()));
        sensorVals.put("Br", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getBr()));

        waitForStart();

        while (opModeIsActive()){

            sensorVals.put("FL", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getFl()));
            sensorVals.put("Fr", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getFr()));
            sensorVals.put("Bl", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getBl()));
            sensorVals.put("Br", 1.0 * bulkData.getMotorCurrentPosition(noodle.getDrivetrain().getBr()));


            telemetry.addData("Y", noodle.getDrivetrain().getSensors().getYDistance());
            telemetry.addData("Angle", noodle.getDrivetrain().getSensors().getFirstAngle());
            telemetry.addData("Current Encoder Average", sensorVals.get("Current Encoder Average"));
            telemetry.addData("FL", sensorVals.get("FL"));
            telemetry.update();
        }
    }
}
