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

import java.util.TreeMap;

@Autonomous(name="Basic: PIDPrototype", group="Linear Opmode")
@Disabled
public class PIDPrototype extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timerPID = new ElapsedTime();
    private DcMotor fr = null;
    private DcMotor fl = null;
    private DcMotor bl;
    private DcMotor br;
    private boolean changeA;
    private double error;
    private double TURN_DEGREES = 90;
    Sensors gyro;
    Drivetrain drivetrainPID;
    private TreeMap<String, Double> sensorVals;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fr  = hardwareMap.get(DcMotor.class, "fr");
        fl  = hardwareMap.get(DcMotor.class, "fl");
        bl  = hardwareMap.get(DcMotor.class, "bl");
        br  = hardwareMap.get(DcMotor.class, "br");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        PID pid = new PID();
        try{
            gyro = new Sensors(this);
            drivetrainPID = new Drivetrain(this, runtime, sensorVals);

        }
        catch(InterruptedException E) {
            telemetry.addLine(E.getMessage());
            RobotLog.i(E.getMessage());
            telemetry.update();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // Tune (Ziegler Nichols)
        // get to oscillate
        // once oscillations, increment D
        // add I as needed -- //TODO: Set Max Sum for Integral Windup
        drivetrainPID.turnPID(0,0,0,0,0,false);



        // run until the end of the match (driver presses STOP)


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl.getPower(), fr.getPower());
            telemetry.update();
        }

}