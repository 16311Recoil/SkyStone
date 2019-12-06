package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "ParkRightC", group = "ControlledGroup")
public class ParkAutoInsideBlue extends LinearOpMode {
    Drivetrain drivetrain;
    Stacker manipulator;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        manipulator = new Stacker(this);

        waitForStart();

        drivetrain.move(0.6,0, 3 * Math.PI / 2,25,2,0.1);

        Thread.sleep(1000);

        drivetrain.move(0.6,0, Math.PI ,1500,2,0.1);

        Thread.sleep(1000);

    }

}
