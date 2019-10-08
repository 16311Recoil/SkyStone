package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.HardwareThread;


public class Autonomous extends LinearOpMode {
    private DangerNoodle robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DangerNoodle(this);



    }
}
