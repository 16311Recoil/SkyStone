package org.firstinspires.ftc.teamcode.BasicAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class BasicTeleOp extends OpMode {
    public DcMotor motorLF;
    public DcMotor motorRF;
    public Servo Arm;


    public void init(){
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        Arm = hardwareMap.servo.get("Arm");

        motorLF.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop(){
        motorLF.setPower(-gamepad1.left_stick_y);
        motorRF.setPower(-gamepad1.right_stick_y);

        if (gamepad2.y)
            Arm.setPosition(0.7);
        if (gamepad2.a)
            Arm.setPosition(0.2);
    }
}



