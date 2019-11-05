package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "AutoTester", group = "ControlledGroup")
public class AutoTester extends LinearOpMode {
    Drivetrain drivetrain;
    Stacker manipulator;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        manipulator = new Stacker(this);

        waitForStart();

        double d1 = drivetrain.inchesToEncoder(24);

        manipulator.setFangs(false);
        Thread.sleep(300);
        drivetrain.move(-0.25, 0, Math.PI/2, 100, 3.2);
        manipulator.setFangs(true);
        Thread.sleep(300);
        drivetrain.move(-0.75, 0, Math.toRadians(310), 1000000, 4);
        manipulator.setFangs(false);

        //drivetrain.move(0.5, 0, Math.toRadians(251.565), 18.97, 4);
        //drivetrain.move(1,0,0,30,6);

        Thread.sleep(300);


    }

}
