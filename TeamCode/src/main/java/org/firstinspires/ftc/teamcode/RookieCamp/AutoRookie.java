package org.firstinspires.ftc.teamcode.RookieCamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Rookiecamp Auto", group = "LinearOpMode")
public class AutoRookie extends LinearOpMode {




    @Override
    public void runOpMode() {
        CustomAuto Auto = new CustomAuto(this);
        waitForStart();
        sleep(1000);
        Auto.moveForward( 0.5, 7);


    }

}
