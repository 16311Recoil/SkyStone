package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "BlueFoundationAuto", group = "ControlledGroup")
public class BlueFoundationAuto extends LinearOpMode {
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

        // Strafe Right slightly
        drivetrain.move(-0.4, 0, Math.PI, 150, 3);
        Thread.sleep(300);

        // Move Forward
        drivetrain.move(0.25, 0, 3 * Math.PI/2, 1400, 5);
        Thread.sleep(1000);
        //Drivetrain.move(-0.5, 0, Math.PI, 25, 4);
        //Thread.sleep(300);
        //Lock FANGS
        manipulator.setFangs(true);
        Thread.sleep(300);
        //Move forward
        drivetrain.move(0.43, 0, Math.PI/2, 2100, 4);
        Thread.sleep(300);
        //Strafe to corner
        drivetrain.move(-0.6, 0, Math.PI, 100, 5);
        Thread.sleep(300);

        // Unlock
        manipulator.setFangs(false);
        Thread.sleep(300);

        // Park (move left)
        drivetrain.move(-0.6, 0, 0, 155, 5);
        Thread.sleep(300);


        //drivetrain.move(0.5, 0, Math.toRadians(251.565), 18.97, 4);
        //drivetrain.move(1,0,0,30,6);

        Thread.sleep(300);
    }

}
