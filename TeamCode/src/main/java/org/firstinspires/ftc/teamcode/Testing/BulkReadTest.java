package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodle.Robot;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "BulkReadTest", group = "Controlled")
public class BulkReadTest extends LinearOpMode {
    private Drivetrain drivetrain;
    private ConcurrentHashMap<String, Double> vals;
    private DangerNoodle r;

    @Override
    public void runOpMode() throws InterruptedException {
        vals = new ConcurrentHashMap<>();
        r = new DangerNoodle(this, false, false);

        waitForStart();

        r.getDrivetrain().move(0.6,0,Math.PI/2,1000,10,0);

        Thread.sleep(10000);

        r.getDrivetrain().moveNoBR(0.6,0,Math.PI/2,1000,10,0);


    }
}