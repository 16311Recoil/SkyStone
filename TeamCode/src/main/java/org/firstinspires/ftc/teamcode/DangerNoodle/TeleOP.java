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
    DangerNoodle teleOp;

    ElapsedTime teleOpTime = new ElapsedTime();

    @Override
    public void init() {
        teleOp = new DangerNoodle(this);
        telemetry.addLine("INIT FINISHED");
        telemetry.update();

    }

    @Override
    public void loop() {
        teleOp.teleopControls();
        telemetry.update();

    }
}
