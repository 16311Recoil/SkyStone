package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@TeleOp
        (name="Lift Test", group="Iterative Opmode")

public class LiftPrototype extends OpMode {

        // Declare OpMode members.
        private ElapsedTime runtime;
        private DcMotor ll; //lift left
        private DcMotor lr; //lift right
        private double power = 0;
        private double gravity = 0.2;
        private boolean lock = false;
        private boolean changeA = false;
        private boolean changeB = false;
        Stacker lift = new Stacker(this);





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


            ll.setMode(RUN_USING_ENCODER);
            Thread.yield();
            lr.setMode(RUN_USING_ENCODER);
            Thread.yield();


            ll.setMode(STOP_AND_RESET_ENCODER);
            Thread.yield();
            lr.setMode(STOP_AND_RESET_ENCODER);
            Thread.yield();

            ll.setMode(RUN_WITHOUT_ENCODER);
            Thread.yield();
            lr.setMode(RUN_WITHOUT_ENCODER);
            Thread.yield();

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
        if (gamepad1.a && !changeA){
            lift.macOut();
        }
        if(gamepad1.b && !changeB){
            lift.macIn();
        }
        changeA = gamepad1.a;
        changeB = gamepad1.b;

    }
    public  double getLiftEncoderAverage() {
        double counter = 0;

        if (ll.getCurrentPosition() == 0) {
            counter += 1;
        }
        if (lr.getCurrentPosition() == 0) {
            counter += 1;
        }
        return ((ll.getCurrentPosition() + lr.getCurrentPosition()) / (2 - counter));
    }
    public void resetLiftEncoders(){
        ll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}


