package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "AutoTester", group = "ControlledGroup")
public class AutoTester extends LinearOpMode {
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());

        waitForStart();


        drivetrain.move(0.5, 0, Math.PI/2, 1500, 4);

        Thread.sleep(300);


    }

}
