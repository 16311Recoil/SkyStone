package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.TreeMap;

@TeleOp (name = "Drivetrain Prototype", group = "Controlled")
public class DrivetrainPrototype extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain tele;
    private Stacker s;
    double heading;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        try {
            tele = new Drivetrain(this, new ElapsedTime(), new TreeMap<String, Double>());
            s = new Stacker(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        heading = tele.getSensors().getFirstAngle();

        telemetry.addData("Init Angle", heading);
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        try {
            tele.turnPID(48,0.7,0.15,0,5, true);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        tele.correctTo(.3, heading,0);

        telemetry.update();
    }
 }


