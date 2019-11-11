package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;

@TeleOp
        (name = "Bitmap Test", group = "Controlled")
//@Disabled
public class BitmapTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BitmapVision bmv = new BitmapVision(this);
        waitForStart();
        telemetry.addData("SkyPOS",bmv.getSkyPos()[0]);
        telemetry.update();

    }
}
