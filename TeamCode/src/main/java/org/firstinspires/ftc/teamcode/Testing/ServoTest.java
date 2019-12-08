package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
        (name = "ServoTest", group = "Controlled")
//@Disabled
public class ServoTest extends LinearOpMode {

    private Servo rFang;
    private Servo lFang;
    private double pos = .0;
    private boolean changeA = false;
    private boolean changeB = false;
    private boolean changeXOR = true;
    @Override
    public void runOpMode() throws InterruptedException {
        rFang = hardwareMap.servo.get("rFang");
        lFang = hardwareMap.servo.get("lFang");

        rFang.setDirection(Servo.Direction.REVERSE);
        lFang.setDirection(Servo.Direction.FORWARD);




        //pincher.setPosition(0);
        //armRotater.setPosition(0.1);
        rFang.setPosition(0);
        lFang.setPosition(0);

        waitForStart();

        while (opModeIsActive()){
            rFang.setPosition(0.32);
            lFang.setPosition(0.32);
            //armRotater.setPosit2on(.5);
            telemetry.addData("Position", rFang.getPosition());
            //telemetry.addData("Position2", armRotater.getPosition());
            telemetry.update();

        }




    }
}
