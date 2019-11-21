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

    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart();
        /*
        // Zero Fangs
        manipulator.setFangs(false);
        Thread.sleep(300);

        // Strafe Right slightly
        drivetrain.move(0.4, 0, Math.PI, 1200, 3);
        Thread.sleep(300);

        // Move Forward
        drivetrain.move(0.2, 0, (3 * Math.PI/2), 1400, 5);

        Thread.sleep(1000);
        //drivetrain.move(-0.5, 0, Math.PI, 25, 4);
        //Thread.sleep(300);
        //Lock FANGS
        manipulator.setFangs(true);
        Thread.sleep(300);

        drivetrain.turnPID(90,(0.86 / 90),0,0.65 / 90,6.5,true);
        Thread.sleep(300);
        //Move forward
        drivetrain.move(0.43, 0, Math.PI/2, 2100, 4);
        Thread.sleep(300);

        drivetrain.move(0.6, 0, Math.PI, 1500, 5);
        Thread.sleep(300);

        // Unlock
        manipulato.setFangs(false);
        Thread.sleep(300);

        drivetrain.move(0.6, 0, 0, 1400, 5);
        Thread.sleep(300);

        // Park (move left)


        //drivetrain.move(0.5, 0, Math.toRadians(251.565), 18.97, 4);
        // drivetrain.move(1,0,0,30,6);
        */
        //drivetrain.correctHeading((0.5 / angle), (0.2/angle),7);

        Thread.sleep(300);

       //drivetrain.move(0.5,0,Math.PI/4,300, 5);

        Thread.sleep(100);

        //drivetrain.move(0.5,0,Math.PI,2500,6);
    }
}
