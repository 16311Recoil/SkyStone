package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

@TeleOp (name = "Drivetrain Prototype", group = "Controlled")
public class DrivetrainPrototype extends OpMode {

        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor fr = null;
        private DcMotor fl = null;
        private DcMotor bl;
        private DcMotor br;
        private boolean changeDpadD;
        private Drivetrain tele;

        @Override
        public void init () {


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
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop () {
        double counter = 0;
        if (gamepad1.dpad_down ^ changeDpadD) {
            counter += 1;
        }
        if ((counter % 2) == 0) { //toggling move tele-op method by pressing down on the d pad
            moveTelop(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else if (gamepad1.a) {   //a moves forward
            fr.setPower(.75);
            fl.setPower(.75);
            bl.setPower(.75);
            br.setPower(.75);
        } else if (gamepad1.b) { //b moves backward
            fr.setPower(-.75);
            fl.setPower(-.75);
            bl.setPower(-.75);
            br.setPower(-.75);
        } else if (gamepad1.x) { //x strafes left
            fr.setPower(.75);
            fl.setPower(-.75);
            bl.setPower(.75);
            br.setPower(-.75);
        } else if (gamepad1.y) { //y strafes right
            fr.setPower(-.75);
            fl.setPower(.75);
            bl.setPower(-.75);
            br.setPower(.75);
        }
        changeDpadD = gamepad1.dpad_down;
    }
        public void moveTelop ( double x, double y, double z){
        fr.setPower(Range.clip(y - x + z, -1, 1));
        fl.setPower(Range.clip(y + x - z, -1, 1));
        br.setPower(Range.clip(y + x + z, -1, 1));
        bl.setPower(Range.clip(y - x - z, -1, 1));
    }
}


