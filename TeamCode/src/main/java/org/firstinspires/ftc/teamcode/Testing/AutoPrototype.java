package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import java.util.TreeMap;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: AutoPrototype", group="Linear Opmode")
public class AutoPrototype extends LinearOpMode {
    Drivetrain dt;
    double heading;

    @Override
    public void runOpMode() throws InterruptedException {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        try {
            dt = new Drivetrain(this, new ElapsedTime(), new TreeMap<String, Double>());
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        heading = dt.getSensors().getFirstAngle();

        telemetry.addData("Init Angle", heading);
        telemetry.update();

        waitForStart();

       // dt.testCorrectTo(heading);
    }
 }
