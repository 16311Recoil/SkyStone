package org.firstinspires.ftc.teamcode.RookieCamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Rookiecamp Auto", group = "LinearOpMode")
public class AutoRookie extends LinearOpMode {


    CustomAuto Auto = new CustomAuto(this);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Auto.moveForward( 0.5, 0.5);
        Auto.jeweldump();

    }

}
