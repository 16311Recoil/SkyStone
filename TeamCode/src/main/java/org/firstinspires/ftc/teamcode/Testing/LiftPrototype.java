package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class LiftPrototype {
    @TeleOp(name="Basic: LiftPrototype", group="Iterative Opmode")
    @Disabled
    public class BasicOpMode_Iterative extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor ll = null; //lift left
        private DcMotor lr = null; //lift right

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            ll  = hardwareMap.get(DcMotor.class, "ll");
            lr  = hardwareMap.get(DcMotor.class, "lr");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            ll.setDirection(DcMotor.Direction.FORWARD);
            lr.setDirection(DcMotor.Direction.FORWARD);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
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
            if (gamepad1.a){
                ll.setPower(.5);
                lr.setPower(.5);
            }
            if (gamepad1.b){
                ll.setPower(0);
                lr.setPower(0);
            }
            if (gamepad1.x){
                ll.setPower(-0.5);
                lr.setPower(-0.5);
            }

        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }
}
