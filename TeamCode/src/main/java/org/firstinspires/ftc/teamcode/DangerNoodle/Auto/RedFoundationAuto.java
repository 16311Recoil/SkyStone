package org.firstinspires.ftc.teamcode.DangerNoodle.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "RedFoundationAuto", group = "ControlledGroup")
public class RedFoundationAuto extends LinearOpMode {
    Drivetrain drivetrain;
    Stacker manipulator;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        manipulator = new Stacker(this);

        waitForStart();

        // Zero Fangs
        manipulator.setFangs(false);
        Thread.sleep(300);

        drivetrain.move(0.4, 0, (3*Math.PI/2), 200, 3,0.1);
        Thread.sleep(300);

        // Strafe Right slightly
        drivetrain.move(0.4, 0, Math.PI, 1400, 3,0.1);
        Thread.sleep(300);

       // double angle = drivetrain.getSensors().getFirstAngle();
        //drivetrain.correctHeading((0.5 / angle), (0.2/angle),2);

        // Move Forward
        drivetrain.move(0.4, 0, (3 * Math.PI/2), 1150, 4,0.1);

        Thread.sleep(300);
        //drivetrain.move(-0.5, 0, Math.PI, 25, 4);
        //Thread.sleep(300);
        //Lock FANGS
        manipulator.setFangs(true);
        Thread.sleep(1100);

        drivetrain.move(0.65,0, Math.PI/2, 1100, 4,0.1);
        Thread.sleep(300);
/*
        drivetrain.turnPID(90,(0.93 / 90),0,0.6 / 90,3,true);
        Thread.sleep(300);
        //drivetrain.move(0.6,0, Math.PI / 2, 2000, 6);
        //Thread.sleep(300);

        drivetrain.move(0.6,0,3 * Math.PI/2,1000,4);
        Thread.sleep(300);


        drivetrain.move(0.9, 0, Math.PI, 6000, 5);
        Thread.sleep(300);


        // Unlock
        manipulator.setFangs(false);
        Thread.sleep(300);


        drivetrain.move(0.5,0,3 * Math.PI/4,100, 5);

        Thread.sleep(100);

        drivetrain.move(0.5,0, 0,2500,6); */
    }
}
