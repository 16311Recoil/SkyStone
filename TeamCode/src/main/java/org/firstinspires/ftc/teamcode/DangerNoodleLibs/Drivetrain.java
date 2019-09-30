package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;



import java.lang.Math;
import java.util.Map;
import java.util.TreeMap;

public class Drivetrain {
    // Instance Variables

    private static final int NUM_MOTORS = 4;
    private LinearOpMode opMode;

    private final double WHEEL_DIAMETER_MM = 100.0;
    private final double WHEEL_DIAMETER_FEET = 0.328084;
    private final double ENCODER_PER_REVOLOUTION = 537.6;

    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 1;
    private final int BACK_LEFT = 2;
    private final int BACK_RIGHT = 3;
    double scale[] = {1, 0.5};
    int a_ButtonCount = 0;

    // Instance Variables
    private ExpansionHubMotor fl, fr, bl, br;
    private ElapsedTime drivetrainClock;

    public Map<String, Double> sensorVals;
    public double[] encoderVals;

    private PID pidControlller;
    private Sensors sensors;
    private final double MAX_POWER = 1;
    private double prevEncoder;
    private boolean isMoving;
    private ExpansionHubEx expansionHub;
    private RevBulkData bulkdata;
    private boolean reset;

    public Drivetrain(LinearOpMode opMode, ElapsedTime timer) throws InterruptedException {
        this.opMode = opMode;
        sensors = new Sensors(opMode);
        encoderVals = new double[4];

        // Tracks Sensor Vals.
        sensorVals = new TreeMap<String, Double>();
        drivetrainClock = new ElapsedTime();


        fl = (ExpansionHubMotor)this.opMode.hardwareMap.dcMotor.get("fl");
        fr = (ExpansionHubMotor)this.opMode.hardwareMap.dcMotor.get("fr");
        bl = (ExpansionHubMotor)this.opMode.hardwareMap.dcMotor.get("bl");
        br = (ExpansionHubMotor)this.opMode.hardwareMap.dcMotor.get("br");


        encoderVals[FRONT_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fl);
        encoderVals[FRONT_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fr);
        encoderVals[BACK_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(bl);
        encoderVals[BACK_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(br);


        sensorVals.put("Current Encoder", getEncoderAverage(encoderVals));
        sensorVals.put("Timestamp", 0.0);

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidControlller = new PID();
        pidControlller.setReset(true);
    }
    /* ============================ UTILITY METHODS ==============================================*/

    /**
     * Method to set all motors to a certain power
     *
     * @param power - power for all motors;
     */
    public void setAllMotors(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);

    }

    /**
     * Basic turn method to turn left or right
     *
     * @param power - power for all motors;
     * @param right - boolean to left or right
     */
    public void turn(double power, boolean right) {
        if (right) {
            fl.setPower(-power);
            fr.setPower(power);
            bl.setPower(-power);
            br.setPower(power);
        } else {
            fl.setPower(power);
            fr.setPower(-power);
            bl.setPower(power);
            br.setPower(-power);
        }
    }
    public void bulkRead(ElapsedTime timer){
        double currTime = timer.milliseconds();
        sensorVals.put("Current Angle", sensors.getFirstAngle());
        sensorVals.put("Previous Encoder", sensorVals.get("Current Encoder"));

        bulkdata = expansionHub.getBulkInputData();

        this.encoderVals[FRONT_LEFT] = bulkdata.getMotorCurrentPosition(fl);
        this.encoderVals[FRONT_RIGHT] = bulkdata.getMotorCurrentPosition(fr);
        this.encoderVals[BACK_LEFT] = bulkdata.getMotorCurrentPosition(bl);
        this.encoderVals[BACK_RIGHT] = bulkdata.getMotorCurrentPosition(br);

        sensorVals.put("Previous Time", sensorVals.get("Timestamp"));
        sensorVals.put("TimeStamp", currTime);
        sensorVals.put("Current Encoder", getEncoderAverage(encoderVals));
    }
    //FIXME : Update getEncoderAverage() to better calibrate encoder movements: create a measure for
    // ensuring that the change of the change in encocer movements- indicates that the encoder is
    // consistently reporting a motion-dependent value rather than a static value or 0 due to wire
    // entanglement, broken encoder, etc,

    public double getEncoderAverage(double[] encoderValues) {
        double encoderAverage = 0;
        int counter = 0;
        for(double encoder: encoderValues) {
            if (encoder == 0){
                counter ++;
            }
            encoderAverage += encoder;
        }
        try{
            return (encoderAverage / (4 - counter));
        } catch(ArithmeticException E){
            reset = true;
            RobotLog.i("All Encoders equal Zero");
            return 0.0;

        }
    }



    /**
     * Move forward using Encoder Feedback
     *
     * @param distance - target distance in feet
     * @param power    - power for all motors
     * @param timeout  - time before timeout
     */
    public void moveForward(double distance, double power, double timeout) {
        double currentPos = getEncoderAverage(encoderVals);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeout && currentPos < feetToEncoder(distance)) { //timer loop to stop motors after time reaches timeout or if destination is reached
            currentPos = sensorVals.get("Current Encoder");
            setAllMotors(power);
        }
        setAllMotors(0);
    }


    /**
     * Move or strafe in the cardinal directions with the option to turn while doing so
     *
     * @param v_d     - Desired Velocity
     * @param v_theta - Desired Rotational Velocity
     * @param angle   - Desired Angle
     */
    public void move(double v_d, double v_theta, double angle, double distance, double timeout) {

        // Calculates required motor powers based on direction of the rollers on the Mecanum wheel,
        // Desired Velocity, Desired Rotational Velocity, and Desired Angle

        // Note that the plane formed by the force vectors of the mecanum wheels rotates the cartesian
        // plane by pi/4, thus creating the shift in the trig function.
        double[] powers = new double[NUM_MOTORS];
        powers[FRONT_LEFT] = v_d * Math.sin(angle + Math.PI / 4) + v_theta;
        powers[FRONT_RIGHT] = v_d * Math.cos(angle + Math.PI / 4) - v_theta;
        powers[BACK_LEFT] = v_d * Math.sin(angle + Math.PI / 4) + v_theta;
        powers[BACK_RIGHT] = v_d * Math.cos(angle + Math.PI / 4) - v_theta;

        // Range of above methods is [-2, 2]; in order to scale to [-1, 1], the maximum is found, and
        // it is used to divide each motor power, conserving the ratio between motor powers, but bringing
        // motor power within desired range.
        double maxPower = powers[0];

        for (int i = 1; i < powers.length; i++)
            maxPower = Math.max(Math.abs(maxPower), Math.abs(powers[i]));


        powers[FRONT_LEFT] /= maxPower;
        powers[FRONT_RIGHT] /= maxPower;
        powers[BACK_LEFT] /= maxPower;
        powers[BACK_RIGHT] /= maxPower;

        // Set Motor Powers for set time
        ElapsedTime timer = new ElapsedTime();
        double currentPos = getEncoderAverage(encoderVals);
        while (currentPos < feetToEncoder(distance) && timer.seconds() < timeout) {
            currentPos = getEncoderAverage(encoderVals);
            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);
        }
        setAllMotors(0);

    }
    // TODO: Account for Mecanum wheel drive
    // Experimentally determine constant multiplier to multiply by; the multiplier will change
    // with the addition of weight on the robot; the current method also does not account for
    // gearing the drive motors.

    /**
     * Converts distance in feet to distance in encoders ticks
     *
     * @param distance - distance in feet
     * @return - distance in encoder ticks
     */
    public double feetToEncoder(double distance) {
        return ENCODER_PER_REVOLOUTION * (distance / WHEEL_DIAMETER_FEET);
    }

    /* ============================ MOVEMENT METHODS =============================================*/

    /**
     * Turning Right or Left using Gyro feedback
     *
     * @param power  - power for all motors
     * @param target - target angle
     * @param right  - boolean to right or left
     */
    public void turnGyro(double power, double target, boolean right) {
        int angle = 0; //TODO: Replacement for getting gyro angles
        while (angle < target && right) {
            turn(power, true);
        }
        while (!right && angle < target) {
            turn(power, false);
        }
    }

    /**
     * PID move straight method:
     * Feedback using gyro.
     * Target is always 0
     *
     * @param encoderDistance - distance from target in encoder ticks
     * @param k_p             - constant for Proportional (P) in PID
     * @param k_i             - constant for Integral (I) in PID
     * @param k_d             - constant for Derivative (D) in PID
     * @param correction      - additional change in output to achieve desired goal
     * @param timeout         - time in seconds before timeout
     **/
    public void movePID(int encoderDistance, double k_p, double k_i, double k_d, double correction, int timeout) {
        ElapsedTime t_i = new ElapsedTime();


    }

    /**
     * PID turning:
     * Feedback using gyro.
     * Target is always 0.
     *
     * @param dTheta  - desired change in angle
     * @param k_p     - constant for Proportional (P) in PID
     * @param k_i     - constant for Integral (I) in PID
     * @param k_d     - constant for Derivative (D) in PID
     * @param timeout - time in seconds before timeout
     * @param right   - boolean to turn left or right
     */
    public void turnPID(double dTheta, double k_p, double k_i, double k_d, int timeout, boolean right) {
        pidControlller.setReset(true);
        pidControlller.setCoeffs(k_p, k_i, k_d);

        double theta_i = sensors.getFirstAngle();
        ElapsedTime t_i = new ElapsedTime();
        pidControlller.setT_i(t_i.seconds());
        pidControlller.setTarget(dTheta);

        while (Math.abs(theta_i - sensors.getFirstAngle()) < dTheta && t_i.seconds() < timeout && opMode.opModeIsActive()) {
            turn(pidControlller.iteration(Math.abs(theta_i - sensors.getFirstAngle()), t_i.seconds()), right);
        }

    }
    /*
     monitorEncoders();
     - Monitors encoders during every bulk read.
     -  change in change in encoder ticks > 0.
     - Value of Encoder ticks != 0.
  */
    public void monitorEncoders(double encoderAverage){


    }


// ================================ Tele-Op Methods =======================================================================

    /**
     * Tele-OP Move method
     *
     * @param x - input variable for strafing - opMode.gamepad1.left_stick_x - x value of the left joystick
     * @param y - input variable for strafing - opMode.gamepad1.left_stick_y - y value of the left joystick
     * @param z - input variable for turning - opMode.gamepad1.right_stick_x - x value of the right joystick
     */
    public void moveTelop(double x, double y, double z) {
        if (opMode.gamepad1.a){
            a_ButtonCount++;                                          //left side -z right side +z
        }
        double scaleSpeed = scale[a_ButtonCount % 2];
        double netTheta = Math.atan2(x,y) - sensors.getFirstAngle();
        double v_d = Math.hypot(x,y);
        fl.setPower( (v_d * (Math.sin( (netTheta) + Math.PI / 4) )) - z );
        fr.setPower( (v_d * (Math.cos( (netTheta) + Math.PI / 4) )) + z );
        bl.setPower( (v_d * (Math.sin( (netTheta) + Math.PI / 4) )) - z );
        br.setPower( (v_d * (Math.cos( (netTheta) + Math.PI / 4) )) + z );
    }

}