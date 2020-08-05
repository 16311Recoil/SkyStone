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
     parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
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

 public double sensrOne ()
 {
     angles = imu.getAngularOrientation();
     return angles.firstAngle;
 }

 public void driveForward (double power, double time)
     {
     ElapsedTime timeone = new ElapsedTime();
     while (timeone.seconds() <= time)
     {
         motorRF.setPower(power);
         motorLF.setPower(power);
         motorLB.setPower(power);
         motorRB.setPower(power);
     }
     zeroPower();
 }

 public void zeroPower()
 {
     motorRF.setPower(0);
     motorLF.setPower(0);
     motorRB.setPower(0);
     motorLB.setPower(0);

 }

 public void turn (double power, boolean right) {
     if (right) {
         motorRF.setPower(-power);
         motorLF.setPower(power);
         motorRB.setPower(-power);
         motorLB.setPower(power);
     } else {
         motorRF.setPower(power);
         motorLF.setPower(-power);
         motorRB.setPower(power);
         motorLB.setPower(-power);
     }

 }

 public void turnGyro(double dTheta, double timeout, double power){
     
     double currentAngle = sensrOne();
     double targetAngle = sensrOne() + dTheta;

     double error = Math.abs(targetAngle - currentAngle);
     ElapsedTime timer = new ElapsedTime();

     while (Math.abs(error) > 0.3 && timer.seconds() < timeout) {

         turn(power, true);

         currentAngle = sensrOne();
         error = Math.abs(targetAngle - currentAngle);
     }

 }
 }






