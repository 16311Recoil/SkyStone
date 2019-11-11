package org.firstinspires.ftc.teamcode.DangerNoodle.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;

@Autonomous
        (name = "RedSkystoneAuto", group = "Controlled")
public class RedSkystone extends LinearOpMode {
    Drivetrain dt;
    BitmapVision bmv;
    Stacker manip;
    private int skyPos;


    @Override
    public void runOpMode() throws InterruptedException {
        dt = new Drivetrain(this, new ElapsedTime(), new ConcurrentHashMap<String, Double>());
        bmv = new BitmapVision(this);
        manip = new Stacker(this);

        while(!isStarted()) {
            skyPos = bmv.getSkyPos()[0];
            telemetry.addData("SKY POS", skyPos);
            telemetry.update();
        }

        waitForStart();

        switch (skyPos){
            case 2:

               dt.move(0.4,0,Math.PI, 700,3);

               Thread.sleep(300);

               dt.move(0.6,0,Math.PI/2, 1100,5);

               Thread.sleep(200);

               manip.setIntakePower(0.5);

               dt.move(0.3,0,Math.PI/2, 100,2);

               Thread.sleep(300);

               manip.setIntakePower(0.8);

               Thread.sleep(3000);

               double angle = dt.getSensors().getFirstAngle();
               //dt.correctHeading((0.5 / angle), (0.2/angle),7);

               Thread.sleep(300);

               dt.move(0.2,0,3* Math.PI/2, 200,5);

               Thread.sleep(300);

               dt.move(0.3,0,0, 1800,5);
               break;

            case 1:
                angle = dt.getSensors().getFirstAngle();
                dt.correctHeading((0.5 / angle), (0.2/angle),7);

                dt.move(0.2,0, Math.PI/2, 1100,5);

                Thread.sleep(300);

                manip.setIntakePower(0.6);

                dt.move(0.14,0,Math.PI/2, 300,3);

                manip.setIntakePower(0.8);

                Thread.sleep(5000);

                dt.move(0.3,0,3* Math.PI/2, 200,3);

                Thread.sleep(300);

                dt.move(0.3,0,Math.PI, 1200,5);


               break;
            case 0:
                dt.move(0.4,0,0,500,3);

                Thread.sleep(300);

                dt.move(0.3,0,Math.PI/2, 1100,5);

                Thread.sleep(200);

                dt.move(0.1,0,Math.PI/2, 100,5);

                Thread.sleep(500);

                Thread.sleep(300);

                manip.setIntakePower(0.6);

                Thread.sleep(3000);

                angle = dt.getSensors().getFirstAngle();
                dt.correctHeading((0.5 / angle), (0.2/angle),7);

                Thread.sleep(300);

                dt.move(0.2,0,3* Math.PI/2, 300,5);

                Thread.sleep(300);

                dt.move(0.3,0,0, 315,5);

                break;
        }

    }
}
