package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Sensors;

@TeleOp(name="Basic: testGyro", group="Iterative Opmode")
@Disabled
public class testGyro extends BasicOpMode_Iterative {

    private double currentPos_1;
    private double pastPos_1;
    private double currentPos_2;
    private double pastPos_2;
    private double currentPos_3;
    private double pastPos_3;
    private boolean changeA;
    private boolean changeB;
    private boolean changeX;
    Sensors gyro1;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init() {
        try{
            gyro1 = new Sensors(this);
        }
        catch (InterruptedException E){
            RobotLog.i(E.getMessage());
        }
    }
    public void getGyro () {
        if (gamepad1.x ^ changeX) {
            currentPos_1 = gyro1.getFirstAngle();
            currentPos_2 = gyro1.getSecondAngle();
            currentPos_3 = gyro1.getThirdAngle();
            telemetry.addData("First Angle", currentPos_1);
            telemetry.addData("Second Angle", currentPos_1);
            telemetry.addData("Third Angle", currentPos_1);
            telemetry.update();
        }
        changeX = gamepad1.x;
    }
    public void changeGyro() {
        if (gamepad1.a ^ changeA) {
            currentPos_1 = gyro1.getFirstAngle();
            currentPos_2 = gyro1.getSecondAngle();
            currentPos_3 = gyro1.getThirdAngle();
        }
        if (gamepad1.b ^ changeB) {
            pastPos_1 = gyro1.getFirstAngle();
            pastPos_2 = gyro1.getSecondAngle();
            pastPos_3 = gyro1.getThirdAngle();
        }
        changeA = gamepad1.a;
        changeB = gamepad1.b;
        telemetry.addData("Original First Angle", currentPos_1);
        telemetry.addData("Original Second Angle", currentPos_1);
        telemetry.addData("Original Third Angle", currentPos_1);
        telemetry.update();
        telemetry.addData("New First Angle", pastPos_1);
        telemetry.addData("New Second Angle", pastPos_2);
        telemetry.addData("New Third Angle", pastPos_3);
        telemetry.update();
        telemetry.addData("Change in First Angle", currentPos_1 - pastPos_1);
        telemetry.addData("Change in Second Angle", currentPos_2 - pastPos_2);
        telemetry.addData("Change in Third Angle", currentPos_3 - pastPos_3);
        telemetry.update();

    }


    @Override
    public void loop() {

    }
    }


