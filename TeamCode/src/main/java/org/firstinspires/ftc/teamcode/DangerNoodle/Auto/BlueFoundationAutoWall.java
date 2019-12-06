package org.firstinspires.ftc.teamcode.DangerNoodle.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;

@Autonomous(name="BlueFoundationAutoC", group="Linear Opmode")

public class BlueFoundationAutoWall extends LinearOpMode{

        public DangerNoodle dangerNoodle;
        private double STRAFE = 4.881889763779528;

    @Override
        public void runOpMode() throws InterruptedException {

            dangerNoodle = new DangerNoodle(this, true,true);

            waitForStart();

            dangerNoodle.moveFoundation(true, false);




          //  dangerNoodle.getDrivetrain().moveStrafeX(0.5,0,3 * Math.PI / 2,24 - STRAFE,2,0.1);


        }
    }

