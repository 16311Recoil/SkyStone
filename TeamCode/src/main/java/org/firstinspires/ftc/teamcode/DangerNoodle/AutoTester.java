package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Sensors;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "AutoTester", group = "ControlledGroup")
public class AutoTester extends LinearOpMode {
    public DangerNoodle dangerNoodle;

    @Override
    public void runOpMode() throws InterruptedException {

        dangerNoodle = new DangerNoodle(this, true, true);


        waitForStart();

        dangerNoodle.testCorrectTo();


    }
}
