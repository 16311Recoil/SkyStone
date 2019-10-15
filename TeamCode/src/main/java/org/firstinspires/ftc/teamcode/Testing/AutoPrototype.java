package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Sensors;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

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
@Disabled
public class AutoPrototype extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fr = null;
    private DcMotor fl = null;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor ll = null; //lift left
    private DcMotor lr = null; //lift right
    private DcMotor il = null;
    private DcMotor ir = null;
    private Drivetrain auto;
    private Stacker intake_outake;
    private TreeMap<String, Double> sensorVal;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sensorVal = new TreeMap<String, Double>();

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
        try{
             auto = new Drivetrain( this, runtime, sensorVal);
             intake_outake = new Stacker(this);
        }
        catch (InterruptedException E){
            RobotLog.i(E.getMessage());
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            /*auto.setAllMotors(1);
            this.sleep(1000);
            auto.setAllMotors(0);
            auto.turn(1,true);
            this.sleep(1000);
            auto.setAllMotors(0);
            intake_outake.setIntakePower(1);
            this.sleep(1000);
            intake_outake.setIntakePower(0);
            intake_outake.setLiftPower(.25);
            this.sleep(250);
            intake_outake.setLiftPower(-.25);
            this.sleep(250);
            intake_outake.setLiftPower(0); */


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", fl, fr);
            telemetry.update();
        }

    }
}