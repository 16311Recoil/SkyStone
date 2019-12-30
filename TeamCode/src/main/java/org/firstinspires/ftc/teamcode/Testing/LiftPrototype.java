package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
        (name="Lift Test", group="Iterative Opmode")

public class LiftPrototype extends OpMode {
    private enum State{
            DIRECT,
            L_SPEED,
            H_SPEED
        }

        // Declare OpMode members.
        private ElapsedTime runtime;
        private DcMotor ll; //lift left
        private DcMotor lr; //lift right
        private double power = 0;
        private double gravity = 0.2;
        private boolean lock = false;




        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            runtime = new ElapsedTime();

            ll  = hardwareMap.get(DcMotor.class, "ll");
            lr  = hardwareMap.get(DcMotor.class, "lr");
            ll.setDirection(DcMotor.Direction.FORWARD);
            lr.setDirection(DcMotor.Direction.REVERSE);

            ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            power += gravity;

            telemetry.addData("Status", "Initialized");
        }

        @Override
        public void init_loop() {
        }

        @Override
        public void start() {
            runtime.reset();
        }


        @Override
        public void loop() {
            liftControlD2();
        }

        @Override
        public void stop() {
        }
    private void liftControlD2() {
        if (gamepad1.right_trigger != 0){
            power = gamepad1.right_trigger;
            ll.setPower(Range.clip(power, -1, 1));
            lr.setPower(Range.clip(power, -1, 1));
            telemetry.addData("POWER", ll.getPower());
        }
        else if (gamepad1.left_trigger != 0){
            ll.setPower(-Range.clip(power, -1, 1));
            lr.setPower(-Range.clip(power, -1, 1));
            telemetry.addData("POWER", -ll.getPower());
        }
        else if (gamepad1.left_bumper){
            ll.setPower(-power);
            lr.setPower(-power);
        }
        else if (gamepad1.right_bumper) {
            ll.setPower(power);
            lr.setPower(power);
        }
        else if(gamepad1.right_stick_button){
            if (lock){
                lock = false;
            }
            lock = true;
        }
        else {
            telemetry.update();
            if (!lock){
                ll.setPower(0);
                lr.setPower(0);
            } else{
                ll.setPower(gravity);
                lr.setPower(gravity);
            }
        }
        telemetry.update();
    }
    public  double getLiftEncoderAverage() {
        double counter = 0;
        if (ll.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (lr.getCurrentPosition() == 0) {
            counter += 1;
        }
        return (ll.getCurrentPosition() + lr.getCurrentPosition() / (2 - counter));
    }
}


