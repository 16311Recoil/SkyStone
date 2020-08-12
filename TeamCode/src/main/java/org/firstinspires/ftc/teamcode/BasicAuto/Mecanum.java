package org.firstinspires.ftc.teamcode.BasicAuto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DangerNoodle.TeleOP;

public class Mecanum extends OpMode {
    public DcMotor RF;
    public DcMotor LF;
    public DcMotor RB;
    public DcMotor LB;


    public void init(){
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");

        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);

    }
    public void loop(){
        double vertical;
        double horizontal;
        double turn;

        while (){
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            RF.setPower(-turn+(vertical-horizontal));
            RB.setPower(-turn+(vertical+horizontal));
            LF.setPower(turn+(vertical+horizontal));
            LB.setPower(turn+(vertical-horizontal));
        }

    }

    }
