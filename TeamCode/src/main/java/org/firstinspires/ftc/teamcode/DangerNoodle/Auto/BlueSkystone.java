package org.firstinspires.ftc.teamcode.DangerNoodle.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DangerNoodle.DangerNoodle;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.BitmapVision;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Drivetrain;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Stacker;

import java.util.concurrent.ConcurrentHashMap;
@Autonomous
        (name = "Blue Skystone Auto", group = "Controlled")
public class BlueSkystone extends LinearOpMode {
    Drivetrain drivetrain;
    BitmapVision vision;
    Stacker manipulator;
    DangerNoodle dangerNoodle;
    private int skyPos;


    @Override
    public void runOpMode() throws InterruptedException {
        dangerNoodle = new DangerNoodle(this, true,true);

        while(!isStarted()) {
            skyPos = 2;
            telemetry.addData("SKY POS", skyPos);
            telemetry.update();
        }

        waitForStart();

        dangerNoodle.skystone(true,true, 0);



//--------------------------------------------------------------------------------------------------------------------------------------------
        /*switch (skyPos){
            case 2:

               dt.move(0.4,0,0, 700,3,0.1);

               Thread.sleep(300);

               dt.move(0.6,0,Math.PI/2, 1100,5,0.1);

               Thread.sleep(200);

               manip.setIntakePower(0.5);

               dt.move(0.3,0,Math.PI/2, 100,2,0.1);

               Thread.sleep(300);

               manip.setIntakePower(0.8);

               Thread.sleep(3000);

               double angle = dt.getSensors().getFirstAngle();
               //dt.correctHeading((0.5 / angle), (0.2/angle),7);

               Thread.sleep(300);

               dt.move(0.2,0,3* Math.PI/2, 200,5,0.1);

               Thread.sleep(300);

               dt.move(0.3,0,Math.PI, 1800,5,0.1);
               break;

            case 1:
                angle = dt.getSensors().getFirstAngle();
                //dt.correctHeading((0.5 / angle), (0.2/angle),7);

                dt.move(0.2,0, Math.PI/2, 1100,5,0.1);

                Thread.sleep(300);

                manip.setIntakePower(0.6);

                dt.move(0.14,0,Math.PI/2, 300,3,0.1);

                manip.setIntakePower(0.8);

                Thread.sleep(5000);

                dt.move(0.3,0,3* Math.PI/2, 200,3,0.1);

                Thread.sleep(300);

                dt.move(0.3,0,Math.PI, 1200,5,0.1);


               break;
            case 0:
                dt.move(0.4,0,Math.PI,500,3,0.1);

                Thread.sleep(300);

                dt.move(0.3,0,Math.PI/2, 1100,5,0.1);

                Thread.sleep(200);

                dt.move(0.1,0,Math.PI/2, 100,5,0.1);

                Thread.sleep(500);

                Thread.sleep(300);

                manip.setIntakePower(0.6);

                Thread.sleep(3000);

                angle = dt.getSensors().getFirstAngle();
                //dt.correctHeading((0.5 / angle), (0.2/angle),7);

                Thread.sleep(300);

                dt.move(0.2,0,3* Math.PI/2, 300,5,0.1);

                Thread.sleep(300);

                dt.move(0.3,0,Math.PI, 315,5,0.1);

                break;


        }*/

    }
}
