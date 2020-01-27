package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodle.Robot;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;

import java.util.Map;
import java.util.TreeMap;
@Disabled
@TeleOp
        (name = "HTP", group = "Controlled")
public class HardwareThreadPrototype extends LinearOpMode {
    DangerNoodle dangerNoodle;
    @Override
    public void runOpMode() throws InterruptedException {
        dangerNoodle = new DangerNoodle(this);
        //hThread = new HardwareThread(dangerNoodle, dummy);
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("FL", dangerNoodle.getSensorVals().get("FL"));
            telemetry.addData("X", dangerNoodle.getDrivetrain().getSensors().getXDistance());
            telemetry.update();
        }
    }

}
