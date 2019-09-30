package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="IntakePrototype", group="Iterative Opmode")
public class IntakePrototype extends OpMode  {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;
    private DcMotor intakeMotor2 = null;
    private double[] power = {-1, -.9, -.8, -.7, -.6, -.5, -.4, -.3, -.2, -1, 0, .1, .2, .3, .4, .5, .6, .7, .8, .9, .1};
    private int counter = 10;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
       if(gamepad1.b) {
           intakeMotor.setPower(power[10]);
           intakeMotor2.setPower(power[10]);
       }
       if(gamepad1.a){
           intakeMotor.setPower(power[15]);
           intakeMotor2.setPower(power[15]);
       }
       if(gamepad1.dpad_up && counter < 20) {
           while (gamepad1.dpad_up){}
           counter += 1;

       }
       if(gamepad1.dpad_down && counter > 0){
           while (gamepad1.dpad_down){}
           counter -= 1;
       }
       intakeMotor.setPower(power[counter]);
       intakeMotor2.setPower(power[counter]);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
