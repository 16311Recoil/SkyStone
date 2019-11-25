package org.firstinspires.ftc.teamcode.DangerNoodle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;

import java.util.concurrent.ConcurrentHashMap;
@TeleOp
        (name = "TeleOP Tester", group = "Controlled")
public class TeleOPTester extends OpMode {

    Drivetrain drivetrain;
    double angle = Math.PI/2;
    private boolean changeDpadUp, changeDpadDown, changeDpadLeft, changeDpadRight;

    @Override
    public void init() {
        try {
            drivetrain = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        changeDpadUp = false;
        changeDpadLeft = false;
        changeDpadRight = false;
        changeDpadDown = false;
    }

    @Override
    public void loop() {
        updateTelemetry();
        controlDrivetrain();
    }

    public void updateTelemetry(){
        String message = "";
        message += ("\nEncoder Average: " + drivetrain.getEncoderAverage(angle));
        message += "\n";
        telemetry.addLine(message);
        telemetry.update();
    }
    public void controlDrivetrain(){
        if (gamepad1.dpad_up && changeDpadUp){
            drivetrain.move(0.5, 0,Math.PI/2,100,5,0.1);
        } else if (gamepad1.dpad_down && changeDpadDown){
            drivetrain.move(0.5, 0 ,3 * Math.PI / 2, 100,5,0.1);
        } else if(gamepad1.dpad_left && changeDpadLeft) {
            drivetrain.move(0.5, 0, Math.PI, 100, 5,0.1);
        } else if(gamepad1.dpad_right && changeDpadRight){
            drivetrain.move(0.5, 0,0,100,5,0.1);
        }
        changeDpadUp = gamepad1.dpad_up;
        changeDpadRight = gamepad1.dpad_right;
        changeDpadLeft = gamepad1.dpad_right;
        changeDpadDown = gamepad1.dpad_down;
    }
}
