package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodle.Robot;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.TreeMap;

@TeleOp
        (name="Basic: LiftPrototype", group="Iterative Opmode")

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
        private Servo gl;
        private Servo gr;
        private boolean changeA;
        private boolean changeB;
        private double power;
        private double liftPower;
        private double intakePower;
        private State currentState;
        private boolean changeX;
        private Drivetrain drivetrain;
        private DcMotor intakeMotor;
        private DcMotor intakeMotor2;
        private boolean changeX2;
        private boolean changeY2;
        private boolean changeA2;
        private boolean changeB2;
        private boolean changeDpadUp1;
        private boolean changeDpadDown1;
        private boolean changeLBumper1;
        private boolean changeRBumper1;
        private boolean changeX1;
        private boolean changeY1;
        private boolean changeDpadLeft2;



        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            try {
                drivetrain = new Drivetrain(this, runtime, new TreeMap<String, Double>());


            } catch (InterruptedException e) {
                telemetry.addLine("Drivetrain Init Failed");
                telemetry.update();
                RobotLog.i(e.getMessage());
            }
            telemetry.addData("Status", "Initialized");
            power = 0;
            liftPower = 0;
            changeA = false;
            changeLBumper1 = false;
            changeRBumper1 = false;
            changeB = false;
            changeX = false;
            changeA2 = false;
            changeB2 = false;
            changeDpadDown1 = false;
            changeDpadUp1 = false;
            runtime = new ElapsedTime();
            intakePower = 0;


            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            ll  = hardwareMap.get(DcMotor.class, "ll");
            lr  = hardwareMap.get(DcMotor.class, "lr");
            gl  = hardwareMap.get(Servo.class, "gl");
            gr = hardwareMap.get(Servo.class, "gr");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            ll.setDirection(DcMotor.Direction.FORWARD);
            lr.setDirection(DcMotor.Direction.REVERSE);


            ll.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            //intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
            //intakeMotor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");

            //intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            //intakeMotor2.setDirection(DcMotor.Direction.FORWARD);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
            currentState = State.DIRECT;


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
            //drivetrain.moveTelop(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //liftControlD1();
            liftControlD2(1);
            gantry();
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }
    private void liftControlD2(double power) {
        if (gamepad1.right_trigger != 0){
            ll.setPower(gamepad1.right_trigger);
            lr.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger != 0){
            ll.setPower(-gamepad1.left_trigger);
            lr.setPower(-gamepad1.left_trigger);
        }
        else if (gamepad1.left_bumper){
            ll.setPower(-power);
            lr.setPower(-power);
        }
        else if (gamepad1.right_bumper) {
            ll.setPower(power);
            lr.setPower(power);
        }
        if ((gamepad1.x && !changeX)){
            telemetry.addData("Encoder:", getLiftEncoderAverage());
        }
        telemetry.update();
        ll.setPower(0);
        lr.setPower(0);
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
    public void gantry(){
        if (gamepad1.a){
            gl.setPosition(1);
            gr.setPosition(1);
        }

    }
}


