package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;

public class BulkReadTest extends LinearOpMode {
    private DangerNoodle robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DangerNoodle(this);
        while (!isStopRequested()){
            for(String a: robot.getSensorVals().keySet()){
                telemetry.addData(a, robot.getSensorVals().get(a));
            }
            telemetry.update();
        }
    }
}