package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.PID;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Sensors;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Map;
import java.util.TreeMap;

@Autonomous(name="PIDPrototype", group="Linear Opmode")

public class PIDPrototype extends LinearOpMode {
    private ElapsedTime runtime;
    private ElapsedTime timerPID;
    private Drivetrain drivetrain;


    @Override
    public void runOpMode() {
        // Declare OpMode members.
        runtime = new ElapsedTime();
        timerPID = new ElapsedTime();
        {
            try {
                drivetrain = new Drivetrain(this, runtime, new TreeMap<String, Double>());
                telemetry.addData("Current Angle", drivetrain.sensorVals.get("Current Angle"));
                telemetry.update();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        waitForStart();

        try {
            drivetrain.turnPID(90,(0.85 / 88),0,(0.2 / 88),5,false);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
}