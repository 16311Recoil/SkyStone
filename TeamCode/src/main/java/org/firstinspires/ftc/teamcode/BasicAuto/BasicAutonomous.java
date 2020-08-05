package org.firstinspires.ftc.teamcode.BasicAuto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class BasicAutonomous {
    public DcMotor motorRF;
    public DcMotor motorLF;
    public DcMotor motorRB;
    public DcMotor motorLB;
    BNO055IMU imu;

    // State used for updating tgelemetry
    Orientation angles;


 public BasicAutonomous (LinearOpMode Auto){
     motorRF = Auto.hardwareMap.dcMotor.get("motorRF");
     motorLF = Auto.hardwareMap.dcMotor.get("motorLF");
     motorRB = Auto.hardwareMap.dcMotor.get("motorRB");
     motorLB = Auto.hardwareMap.dcMotor.get("motorLB");



     BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
     parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
     parameters.accelUnit = BNO55IMU.AccelUnit.METERS_PERSEC_PERSEC;
     parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
     parameters.loggingEnabled = true;
     parameters.loggingTag = "IMU";
     parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

     // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
     // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
     // and named "imu".
     imu = Auto.hardwareMap.get(BNO055IMU.class, "imu");
     imu.initialize(parameters);

 }
 double Drive_Power = 1.0;
 public double Sensorone ()
 {
     return angles.firstAngle;
 }

 public void DriveForward (double power, double time)
 {
     ElapsedTime timeone = new ElapsedTime();
     while (timeone.seconds() < time || timeone.seconds() == time)
     {
         motorRF.setPower(power);
         motorLF.setPower(power);
     }
     StopDriving();
 }

 public void DriveBackward (double power, double time)
 {
     ElapsedTime timeone = new ElapsedTime();
     while (timeone.seconds() < time || timeone.seconds() == time)
     {
         motorRF.setPower(power);
         motorLF.setPower(power);
     }
     StopDriving();
 }


 public void StopDriving()
 {
     motorRF.setPower(0);
     motorLF.setPower(0);

 }

 public void Turn (double power, double angle) {
     while (angles.firstAngle > angle || angles.firstAngle == angle) {
         motorRF.setPower(-power);
         motorLF.setPower(power);
     }
     StopDriving();

 }
}





