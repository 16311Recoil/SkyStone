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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        waitForStart();
        runtime.reset();

        PID pid = new PID();
        try{
            gyro = new Sensors(this);
            drivetrainPID = new Drivetrain(this, runtime);

        }
        catch(InterruptedException E) {
            RobotLog.i(E.getMessage());
        }


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a ^ changeA) {
                double target = gyro.getFirstAngle() + TURN_DEGREES;
                error = TURN_DEGREES;
                while (error > 0){
                    drivetrainPID.turn(pid.iteration(error, runtime.seconds()), true);
                    error = target - gyro.getFirstAngle();
                }
                drivetrainPID.setAllMotors(0);
            }
            changeA = gamepad1.a;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl, fr);
            telemetry.update();
        }

    }
}