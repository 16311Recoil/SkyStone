package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OutakePrototype", group="Iterative Opmode")
public class OutakePrototype extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo pincher = null;
    private Servo armRotater;
    private boolean changeA = false;
    private boolean changeB = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        pincher  = hardwareMap.get(Servo.class, "pincher");
        armRotater = hardwareMap.get(Servo.class, "armRotater");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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
        double counterA = 0;
        double counterB = 0;
        if (gamepad1.a ^ changeA){
            counterA += 1;
        }
        if ((counterA % 2) == 0){
           pincher.setPosition(0);
        }
        else{
            pincher.setPosition(90);
        }
        if (gamepad1.b ^ changeB){
            counterB ++;
        }
        if ((counterB % 2) == 0){
            armRotater.setPosition(0);
        }
        else {
            armRotater.setPosition(180);
        }
        changeA = gamepad1.a;
        changeB = gamepad1.b;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
