package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Testing.LiftPrototype;

public class Sensors {
    private LinearOpMode opMode;
    private BasicOpMode_Iterative iterative;

    //REV2Distance Sensor?
    // Gyro Declaration
    public BNO055IMU gyro;
    private Orientation angles;
    Acceleration gravity;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public Sensors(LinearOpMode opMode) throws InterruptedException{
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
    public Sensors(BasicOpMode_Iterative opMode) throws InterruptedException{
        iterative = opMode;

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
}
