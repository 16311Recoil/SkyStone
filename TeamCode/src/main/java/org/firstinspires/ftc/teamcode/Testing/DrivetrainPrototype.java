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
    private DcMotor fr = null;
    private DcMotor fl = null;
    private DcMotor bl;
    private DcMotor br;
    private boolean changeDpadD;
    private boolean changeA;
    private boolean changeB;
    private boolean changeX;
    private boolean changeY;
    public BNO055IMU gyro;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    private Drivetrain tele;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        changeA = false;
        changeB = false;
        changeX = false;
        changeDpadD = false;
        changeY = false;
        try {
            tele = new Drivetrain(this, new ElapsedTime(), new TreeMap<String, Double>());
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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
        double counter = 0;
        if (gamepad1.dpad_down ^ changeDpadD) {
            counter += 1;
        }
        if ((counter % 2) == 0) { //toggling move tele-op method by pressing down on the d pad
            tele.moveTelop(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }else if (gamepad1.a ^ changeA) {   //a moves forward
                fr.setPower(.75);
                fl.setPower(.75);
                bl.setPower(.75);
                br.setPower(.75);
            } else if (gamepad1.b ^ changeB) { //b moves backward
                fr.setPower(-.75);
                fl.setPower(-.75);
                bl.setPower(-.75);
                br.setPower(-.75);

            } else if (gamepad1.x ^ changeX) { //x strafes left
                fr.setPower(.75);
                fl.setPower(-.75);
                bl.setPower(.75);
                br.setPower(-.75);
            } else if (gamepad1.y ^ changeY) { //y strafes right
                fr.setPower(-.75);
                fl.setPower(.75);
                bl.setPower(-.75);
                br.setPower(.75);

            }
            changeA = gamepad1.a;
            changeB = gamepad1.b;
            changeX = gamepad1.x;
            changeY = gamepad1.y;
            changeDpadD = gamepad1.dpad_down;
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() { }
        public void moveTelop ( double x, double y, double z){

            fr.setPower(Range.clip(y - x - z, -1, 1));
            fl.setPower(Range.clip(y + x + z, -1, 1));
            br.setPower(Range.clip(y + x - z, -1, 1));
            bl.setPower(Range.clip(y - x + z, -1, 1));
        }

    }


