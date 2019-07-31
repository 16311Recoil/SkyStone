package org.firstinspires.ftc.teamcode.RookieCamp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class CustomAuto {
    public DcMotor LM;
    public DcMotor RM;
    private DcMotor BL = null;
    ElapsedTime timer = new ElapsedTime();
    double lastAngle;
    LinearOpMode opMode;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating tgelemetry
    Orientation angles;

    public CustomAuto(LinearOpMode AutoRookie) {
        LM = AutoRookie.hardwareMap.dcMotor.get("LM");
        RM = AutoRookie.hardwareMap.dcMotor.get("RM");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
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
        imu = AutoRookie.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

    }

    public void moveForward(double power, double time) {
        timer.reset();
        while (timer.seconds() < time) {
            LM.setPower(power);
            RM.setPower(-power);
        }

    }

    public void moveBackward(double power, double time) {
        timer.reset();
        while (timer.seconds() < time) {
            LM.setPower(-power);
            RM.setPower(power);
        }
    }

    public void turn(double power, double degrees) {
        double reset = -getAngle();
        if (degrees + reset > 0) {
            while (getAngle() < degrees  + reset) {
                LM.setPower(power);
                RM.setPower(-power);
            }
        } else {
            while (getAngle() < degrees  + reset) {
                LM.setPower(-power);
                RM.setPower(power);
            }
        }

    }
    public void jeweldump(){
        moveForward(.5,3);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BL.setPower(.5);
        wait(1000);
        BL.setDirection(DcMotor.Direction.REVERSE);
        wait(1000);
        BL.setPower(0);

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        opMode.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            }
        });

        opMode.telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        opMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });


    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getAngle() {
        return angles.firstAngle;
    }

}


