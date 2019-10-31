package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
        (name = "ServoTest", group = "Controlled")

public class ServoTest extends LinearOpMode {

    private Servo lFang;
    private Servo rFang;
    private double pos = .0;
    private boolean changeA = false;
    private boolean changeB = false;
    private boolean changeXOR = true;
    @Override
    public void runOpMode() throws InterruptedException {
        lFang = hardwareMap.servo.get("lFang");
        rFang = hardwareMap.servo.get("rFang");

        lFang.setDirection(Servo.Direction.FORWARD);
        rFang.setDirection(Servo.Direction.REVERSE);




        lFang.setPosition(0);
        rFang.setPosition(0);

        waitForStart();

        while (opModeIsActive()){
            lFang.setPosition(0.3);
            rFang.setPosition(0.3);
            telemetry.addData("Position", lFang.getPosition());
            telemetry.addData("Position2", rFang.getPosition());
            telemetry.update();

        }




    }
}
