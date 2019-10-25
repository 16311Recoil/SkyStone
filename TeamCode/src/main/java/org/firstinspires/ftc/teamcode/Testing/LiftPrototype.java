package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DangerNoodle.Robot;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;

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

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            ll.setDirection(DcMotor.Direction.FORWARD);
            lr.setDirection(DcMotor.Direction.REVERSE);

            intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
            intakeMotor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");

            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
            intakeMotor2.setDirection(DcMotor.Direction.FORWARD);

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
            intakeControlD1();
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }
        private double h_scale_speed(double v_d) {
            return Range.clip(0.0308 * Math.exp(4.3891 * v_d), 0.05, 1);
        }
        private double l_scale_speed(double input){
            // 0.3253ln(x) + 0.9069
            return Range.clip(0.3243 * Math.log(input) + 0.9069, 0.05, 1);
        }
        private double calculatePower(double input){
            if (currentState == State.DIRECT)
                return Range.clip(input, -1, 1);
            else if (currentState == State.H_SPEED)
                return Range.clip(h_scale_speed(input), -1, 1);
            else if (currentState == State.L_SPEED)
                return Range.clip(l_scale_speed(input), -1, 1);
            return 0;
        }
        private void intakeControlD2(){

            if(gamepad2.x ^ changeX2 && intakePower <= 1) {
                intakePower += .1;
            }
            else if(gamepad2.y ^ changeY2 && intakePower >= -1){
                intakePower -= .1;
            }
            intakeMotor.setPower(Range.clip(intakePower, -1,1));
            intakeMotor2.setPower(-Range.clip(intakePower, -1,1));
            changeX2 = gamepad2.x;
            changeY2 = gamepad2.y;
        }
        private void intakeControlD1(){
            if(gamepad1.x ^ changeX1 && intakePower <= 1) {
                intakePower += 0.1;
            }
            else if(gamepad1.y ^ changeY1 && intakePower >= -1){
                if(intakePower == 0.1)
                    intakePower = 0;
                else
                    intakePower -= 0.1;

            }
            telemetry.addData("Intake Power: ",intakePower);
            telemetry.update();
            intakeMotor.setPower(Range.clip(intakePower, -1,1));
            intakeMotor2.setPower(-Range.clip(intakePower, -1,1));

            changeX1 = gamepad1.x;
            changeY1 = gamepad1.y;


        }
        private void liftControlD1(){

            if (gamepad1.dpad_down && gamepad1.dpad_up)
                liftPower = 0;
            else if (gamepad1.dpad_up ^ changeDpadUp1)
                liftPower = 0.75;
            else if(gamepad1.dpad_down ^ changeDpadDown1)
               liftPower = -0.75;

            ll.setPower(liftPower);
            lr.setPower(liftPower);

            changeDpadDown1 = gamepad1.dpad_down;
            changeDpadUp1 = gamepad1.dpad_up;
        }
        private void liftControlD2() {

            if (gamepad2.a ^ changeA2){
                currentState = State.L_SPEED;
            }
            else if (gamepad2.b ^ changeB2) {
                currentState = State.H_SPEED;
            }
            else if (gamepad2.dpad_left ^ changeDpadLeft2)
                currentState = State.DIRECT;

            changeA = gamepad2.a;
            changeB = gamepad2.b;
            changeDpadLeft2 = gamepad2.dpad_left;

            ll.setPower(gamepad2.left_stick_y);
            lr.setPower(gamepad2.left_stick_y);
        }
}


