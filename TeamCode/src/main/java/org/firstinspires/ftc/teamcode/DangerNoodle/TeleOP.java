package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.TreeMap;

@TeleOp
        (name = "TeleOp", group = "controlled")
public class TeleOP extends OpMode {
    //DangerNoodle teleOp;
    Drivetrain drivetrain;
    Stacker stacker;
    ElapsedTime teleOpTime = new ElapsedTime();

    @Override
    public void init() {
        try {
            drivetrain = new Drivetrain(this, new ElapsedTime(), new TreeMap<String, Double>());
            stacker = new Stacker(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void loop() {
        drivetrain.moveTelop2(gamepad1.right_stick_x, -gamepad1.right_stick_y, gamepad1.left_stick_x);
        drivetrain.checkState();
        stacker.stackerTeleControl(0.85,1,1);
        telemetry.update();

    }
}
