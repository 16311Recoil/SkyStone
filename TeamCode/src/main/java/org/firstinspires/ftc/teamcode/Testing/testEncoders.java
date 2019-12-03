package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.TreeMap;

@Autonomous(name="Basic: 0AutoPrototype", group="Linear Opmode")

public class testEncoders extends LinearOpMode{

        public DangerNoodle dangerNoodle;
        private double STRAFE = 4.881889763779528;

    @Override
        public void runOpMode() throws InterruptedException {

            dangerNoodle = new DangerNoodle(this, true,true);

            waitForStart();

            dangerNoodle.moveFoundation(true);




          //  dangerNoodle.getDrivetrain().moveStrafeX(0.5,0,3 * Math.PI / 2,24 - STRAFE,2,0.1);


        }
    }

