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

@Autonomous(name="Basic: PIDPrototype", group="Linear Opmode")
@Disabled
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
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        drivetrain.turnPID(0,0,0,0,0,false);


    }
}