package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
@Autonomous
        (name = "BulkReadTest", group = "Controlled")
public class BulkReadTest extends LinearOpMode {
    private DangerNoodle robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DangerNoodle(this);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addLine(robot.toString());
            telemetry.update();
        }
    }
}