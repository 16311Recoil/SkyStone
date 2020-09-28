package org.firstinspires.ftc.teamcode.BasicAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Autonomous extends LinearOpMode {
    public void runOpMode(){
        BasicAutonomous Auto = new BasicAutonomous(this);
        waitForStart();
        Auto.DriveForward(1,4);
        Auto.Turn(1,90);
    }
}
