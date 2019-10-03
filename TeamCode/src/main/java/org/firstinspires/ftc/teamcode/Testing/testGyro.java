package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DangerNoodleLibs.Sensors;

@TeleOp(name="Basic: testGyro", group="Iterative Opmode")
@Disabled
public class testGyro extends LinearOpMode{
    private LinearOpMode opMode;
    //REV2Distance Sensor?
    // Gyro Declaration
    public BNO055IMU gyro;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private double currentPos_1;
    private double pastPos_1;
    private double currentPos_2;
    private double pastPos_2;
    private double currentPos_3;
    private double pastPos_3;

    public testGyro(LinearOpMode opMode) throws InterruptedException{
        this.opMode = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = this.opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

    }

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }

    }

    // Updates Gyro within gyro methods
    private void updateGyro()
    {
        angles = gyro.getAngularOrientation();
    }

    // Bulk Reads Sensor Data to update all values for use.
    public void updateSensorVals(){
        updateGyro();

    }
    // Determine which is yaw, pitch, and roll.
    public double getFirstAngle()
    {
        updateGyro();
        return angles.firstAngle;
    }
    public double getSecondAngle()
    {
        updateGyro();
        return angles.secondAngle;
    }
    public double getThirdAngle()
    {
        updateGyro();
        return angles.thirdAngle;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void getGyro () {
        currentPos_1 = getFirstAngle();
        currentPos_2 = getSecondAngle();
        currentPos_3 = getThirdAngle();
        System.out.printf("%.2f", currentPos_1);
        System.out.printf("%.2f", currentPos_2);
        System.out.printf("%.2f", currentPos_3);//TODO Don't know how to output doubles to the robotLog
    }
    public void changeGyro() {
        if (gamepad1.a) {
            currentPos_1 = getFirstAngle();
            currentPos_2 = getSecondAngle();
            currentPos_3 = getThirdAngle();
        }
        if (gamepad1.b) {
            pastPos_1 = getFirstAngle();
            pastPos_2 = getSecondAngle();
            pastPos_3 = getThirdAngle();
        }
        System.out.printf("%.2f", currentPos_1);  //TODO: Again don't know how to output doubles to robot log
        System.out.printf("%.2f", currentPos_2);
        System.out.printf("%.2f", currentPos_3);
        System.out.println();
        System.out.printf("%.2f", pastPos_1);
        System.out.printf("%.2f", pastPos_2);
        System.out.printf("%.2f", pastPos_3);
        System.out.println();
        System.out.printf("%.2f", currentPos_1 - pastPos_1);
        System.out.printf("%.2f", currentPos_2 - pastPos_2);
        System.out.printf("%.2f", currentPos_3 - pastPos_3);

    }
}
