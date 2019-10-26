package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;

import java.util.TreeMap;

@TeleOp(name = "TeleOp", group = "controlled")
public class TeleOP extends OpMode {
    DangerNoodle r;
    ElapsedTime teleOpTime = new ElapsedTime();

    @Override
    public void init() {
        r = new DangerNoodle(this);


    }

    @Override
    public void loop() {
        r.teleOpControl();
    }
}
