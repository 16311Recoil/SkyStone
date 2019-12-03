package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
        (name = "ServoTest", group = "Controlled")
//@Disabled
public class ServoTest extends LinearOpMode {

    private Servo armRotater;
    private Servo pincher;
    private double pos = .0;
    private boolean changeA = false;
    private boolean changeB = false;
    private boolean changeXOR = true;
    @Override
    public void runOpMode() throws InterruptedException {
        pincher = hardwareMap.servo.get("pincher");
        //armRotater = hardwareMap.servo.get("gr");

        pincher.setDirection(Servo.Direction.REVERSE);




        //pincher.setPosition(0);
        //armRotater.setPosition(0.1);
        pincher.setPosition(0);

        waitForStart();

        while (opModeIsActive()){
            pincher.setPosition(0.8);
            //armRotater.setPosition(.5);
            telemetry.addData("Position", pincher.getPosition());
            //telemetry.addData("Position2", armRotater.getPosition());
            telemetry.update();

        }




    }
}
