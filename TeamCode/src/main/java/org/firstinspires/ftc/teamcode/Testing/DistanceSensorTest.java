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

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.RecursiveAction;
@TeleOp
        (name = "Sensors Test 2", group = "Controlled")
public class DistanceSensorTest extends LinearOpMode {

    HardwareThread thread;
    DangerNoodle noodle;
    ConcurrentHashMap<String, Double> sensorVals;
    @Override
    public void runOpMode() throws InterruptedException {
        sensorVals = new ConcurrentHashMap<>();
        noodle = new DangerNoodle(this);

        waitForStart();

        while (opModeIsActive()){

            telemetry.addData("X", noodle.getSensorVals().get("X"));
            telemetry.addData("Y", noodle.getSensorVals().get("Y"));
            telemetry.addData("Angle", noodle.getDrivetrain().getSensors().getFirstAngle());
            telemetry.addData("Current Encoder Average", sensorVals.get("Current Encoder Average"));
            telemetry.addData("FL", sensorVals.get("FL"));
            telemetry.update();
        }
    }
}
