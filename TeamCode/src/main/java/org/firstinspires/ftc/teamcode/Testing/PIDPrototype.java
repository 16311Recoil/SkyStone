package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    private double k_i;
    private double k_p;
    private double k_d;
    private double t_sum;
    private double previousError;
    private double p = 0;
    private double i = 0;
    private double d = 0;

    private double target;
    private boolean reset;
    private double t_i;

    private double MAX_SUM = 0.0; // test sum

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.a) {
                iterationPID(90, timerPID.seconds() ) ; //TODO Check with Anish what error is measured in
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl, fr);
            telemetry.update();
        }

    }
    //TODO Get Anish to double check the PID and make sure that it works as intended
    public double iterationPID(double error, double currentTime) {
        double deltaTime = currentTime - t_i;
        p = k_p * error;
        t_sum = 0.5 * (error + previousError) * deltaTime;
        if (t_sum > MAX_SUM) {
            t_sum = MAX_SUM;// test for maxSum
        }
        i = k_i * t_sum;
        d = k_d * (error - previousError) / deltaTime;
      // Add this??  t_i = currentTime;
        return (p + i + d);
    }
    }

    }
}