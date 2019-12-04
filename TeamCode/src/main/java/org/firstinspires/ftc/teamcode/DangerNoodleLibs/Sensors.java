package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Testing.LiftPrototype;

public class Sensors {
    private LinearOpMode opMode;
    private OpMode iterative;

    // REV 2 Distance Sensor
    private DistanceSensor X_distanceSensor;
    private DistanceSensor Y_distanceSensor;

    private Rev2mDistanceSensor X_sensorTimeOfFlight;
    private Rev2mDistanceSensor Y_sensorTimeOfFlight;

    // Gyro Declaration
    public BNO055IMU gyro;
    private Orientation angles;
    private Acceleration gravity;
    private BNO055IMU.Parameters parameters;



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

        X_distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "xDistance");
        Y_distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "yDistance");

        X_sensorTimeOfFlight = (Rev2mDistanceSensor)X_distanceSensor;
        Y_sensorTimeOfFlight = (Rev2mDistanceSensor)Y_distanceSensor;
    }
    public Sensors(OpMode opMode) throws InterruptedException{
        iterative = opMode;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = this.iterative.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);

        X_distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "xDistance");
        Y_distanceSensor = opMode.hardwareMap.get(DistanceSensor.class, "yDistance");

        X_sensorTimeOfFlight = (Rev2mDistanceSensor)X_distanceSensor;
        Y_sensorTimeOfFlight = (Rev2mDistanceSensor)Y_distanceSensor;

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
    public double getXDistance(){
        return X_distanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getYDistance() {
        return Y_distanceSensor.getDistance(DistanceUnit.INCH);
    }

}
