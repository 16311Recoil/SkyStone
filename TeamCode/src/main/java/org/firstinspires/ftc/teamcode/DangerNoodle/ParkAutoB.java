package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "BlueAutoPark", group = "ControlledGroup")
public class ParkAutoB extends LinearOpMode {
    Drivetrain drivetrain;
    Stacker manipulator;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        manipulator = new Stacker(this);

        waitForStart();

        drivetrain.move(0.4, 0, Math.PI/2, 1200, 5,0.1);
        Thread.sleep(400);
        drivetrain.move(0.4, 0, 0, 600, 10,0.1);
        /*manipulator.setFangs(false);
        Thread.sleep(300);
        drivetrain.move(-0.2, 0, Math.PI/2, 1300, 5);
        Thread.sleep(300);
        //drivetrain.move(-0.5, 0, Math.PI, 25, 4);
        //Thread.sleep(300);
        manipulator.setFangs(true);
        Thread.sleep(300);
        drivetrain.move(0.3, 0, Math.PI/2, 1200, 4);
        Thread.sleep(300);
        manipulator.setFangs(false);
        */


        //drivetrain.move(0.5, 0, Math.toRadians(251.565), 18.97, 4);
        //drivetrain.move(1,0,0,30,6);

        Thread.sleep(300);


    }

}
