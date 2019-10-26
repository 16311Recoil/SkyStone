package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;



import java.lang.Math;
import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;

public class Drivetrain {
    private enum State{
       FULL_SPEED,
        HALF_SPEED,
        H_SCALE_POWER,
        L_SCALE_POWER;
    }
    // Instance Variables
    private State currentState;

    private static final int NUM_MOTORS = 4;
    public LinearOpMode opMode;
    public OpMode opMode_iterative;

    private double INCH_PER_ENCODER = 40;


    private final int FRONT_LEFT = 0;
    private final int FRONT_RIGHT = 1;
    private final int BACK_LEFT = 2;
    private final int BACK_RIGHT = 3;
    double scale[] = {1, 0.5};
    int dpadd_ButtonCount = 0;

    // Instance Variables

    private DcMotor fl, fr, bl, br;
    private ElapsedTime drivetrainClock;
    public Map<String, Double> sensorVals;
    public double[] encoderVals;
    private PID pidControlller;
    private Sensors sensors;
    private double maxPower = 1;
    private double minPower = 0.05;
    private double prevEncoder;
    private boolean isMoving;
    //private ExpansionHubEx expansionHub;
    ///private RevBulkData bulkdata;
    private boolean reset;
    private double multiplier;


    public Drivetrain(LinearOpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {


        this.opMode = opMode;
        sensors = new Sensors(this.opMode);
        encoderVals = new double[4];

        // Tracks Sensor Vals.
        this.sensorVals = sensorVals;
        drivetrainClock = new ElapsedTime();
        currentState = State.FULL_SPEED;

        fl = this.opMode.hardwareMap.dcMotor.get("fl");
        fr = this.opMode.hardwareMap.dcMotor.get("fr");
        bl = this.opMode.hardwareMap.dcMotor.get("bl");
        br = this.opMode.hardwareMap.dcMotor.get("br");

       // encoderVals[FRONT_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fl);
       // encoderVals[FRONT_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fr);
       // encoderVals[BACK_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(bl);
       // encoderVals[BACK_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(br);


        sensorVals.put("Current Encoder", getEncoderAverage(encoderVals));
        sensorVals.put("Timestamp", 0.0);

        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);


        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidControlller = new PID();
        pidControlller.setReset(true);
        multiplier = 1;

        opMode.telemetry.addLine("Drivetrain Init Completed");
        opMode.telemetry.update();
    }
    public Drivetrain(OpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {
        this.opMode_iterative = opMode;
        sensors = new Sensors(this.opMode_iterative);
        encoderVals = new double[4];

        // Tracks Sensor Vals.
        this.sensorVals = sensorVals;
        drivetrainClock = new ElapsedTime();

        currentState = State.FULL_SPEED;

        fl = this.opMode_iterative.hardwareMap.dcMotor.get("fl");
        fr = this.opMode_iterative.hardwareMap.dcMotor.get("fr");
        bl = this.opMode_iterative.hardwareMap.dcMotor.get("bl");
        br = this.opMode_iterative.hardwareMap.dcMotor.get("br");


        //encoderVals[FRONT_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fl);
        //encoderVals[FRONT_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fr);
        //encoderVals[BACK_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(bl);
        //encoderVals[BACK_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(br);


        sensorVals.put("Current Encoder", getEncoderAverage(encoderVals));
        sensorVals.put("Timestamp", 0.0);

        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidControlller = new PID();
        pidControlller.setReset(true);
        multiplier = 1;
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

    public void resetEncoders(){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
    }
    public void getEncoders(){
        encoderVals[FRONT_LEFT] = fl.getCurrentPosition();
        encoderVals[FRONT_RIGHT] = fr.getCurrentPosition();
        encoderVals[BACK_LEFT] = bl.getCurrentPosition();
        encoderVals[BACK_RIGHT] = br.getCurrentPosition();
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

    public State getCurrentState() {
        return currentState;
    }

    public void setCurrentState(State currentState) {
        this.currentState = currentState;
    }

    public DcMotor getFl() {
        return fl;
    }

    public void setFl(DcMotor fl) {
        this.fl = fl;
    }

    public DcMotor getFr() {
        return fr;
    }

    public void setFr(DcMotor fr) {
        this.fr = fr;
    }

    public DcMotor getBl() {
        return bl;
    }

    public void setBl(DcMotor bl) {
        this.bl = bl;
    }

    public DcMotor getBr() {
        return br;
    }

    public void setBr(DcMotor br) {
        this.br = br;
    }

    public double[] getEncoderVals() {
        return encoderVals;
    }

    public void setEncoderVals(double[] encoderVals) {
        this.encoderVals = encoderVals;
    }

    public void setBr(ExpansionHubMotor br) {
        this.br = br;
    }
    public Sensors getSensors() {
        return sensors;
    }

    public void setSensors(Sensors sensors) {
        this.sensors = sensors;
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
            return encoderAverage;

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
        resetEncoders();
        double currentPos = getEncoderAverage(encoderVals);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeout && currentPos < inchesToEncoder(distance)) { //timer loop to stop motors after time reaches timeout or if destination is reached
            getEncoders();
            currentPos = getEncoderAverage(encoderVals);
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
        resetEncoders();

        // Calculates required motor powers based on direction of the rollers on the Mecanum wheel,
        // Desired Velocity, Desired Rotational Velocity, and Desired Angle

        // Note that the plane formed by the force vectors of the mecanum wheels rotates the cartesian
        // plane by pi/4, thus creating the shift in the trig function.
        double[] powers = new double[NUM_MOTORS];
        powers[FRONT_LEFT] = (v_d * Math.sin(angle)  + v_d * Math.cos(angle) - v_theta);
        powers[FRONT_RIGHT] = (v_d * Math.sin(angle)  - v_d * Math.cos(angle) + v_theta);
        powers[BACK_LEFT] = (v_d * Math.sin(angle)  - v_d * Math.cos(angle) - v_theta);
        powers[BACK_RIGHT] = (v_d * Math.sin(angle)  + v_d * Math.cos(angle) + v_theta);

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

        opMode.telemetry.addData("Powers", Arrays.toString(powers));
        opMode.telemetry.update();


        // Set Motor Powers for set time
        ElapsedTime timer = new ElapsedTime();
        getEncoders();
        opMode.telemetry.addData("Encoder", Arrays.toString(encoderVals));
        opMode.telemetry.update();
        double currentPos = getEncoderAverage(encoderVals);

        while (currentPos < distance && timer.seconds() < timeout & opMode.opModeIsActive()) {

            opMode.telemetry.addData("Inside Loop", Arrays.toString(encoderVals));
            opMode.telemetry.update();
            getEncoders();



            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);
            LynxModule k;

            currentPos = getEncoderAverage(encoderVals);
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
    public double inchesToEncoder(double distance) {
        return distance * INCH_PER_ENCODER;
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
        double error = Math.abs(theta_i - sensors.getFirstAngle());

        while (error < dTheta && t_i.seconds() < timeout && opMode.opModeIsActive()) {
            turn(pidControlller.iteration(error, t_i.seconds()), right);
            error = Math.abs(theta_i - sensors.getFirstAngle());

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

    /**
     *
     * @param  - inputted number to angleWrap in degrees
     * @param radians - boolean on whether to output in radians or degrees
     * @return
     */
    public double angleWrap(double input, boolean radians){
        if (input > 180 ){
            input -= 360;
        }
        if (radians){
            input = Math.toRadians(input);
        }
        return input;
    }


// ================================ Tele-Op Methods =======================================================================


    /**
     * Tele-OP Move method
     *
     * @param x - input variable for strafing - opMode.gamepad1.left_stick_x - x value of the left joystick
     * @param y - input variable for strafing - o
     *          pMode.gamepad1.left_stick_y - y value of the left joystick
     * @param z - input variable for turning - opMode.gamepad1.right_stick_x - x value of the right joystick
     */
    public void moveTelop(double x, double y, double z) {
        double v_d = Math.hypot(x,y) * Math.signum(x) * Math.signum(y);
        double netTheta = Math.atan2(x,y);
        if (v_d < 0.05)
            v_d = 0;
        if (v_d > 0.95)
            v_d = 1;
        checkState();

        if (currentState.equals(State.FULL_SPEED))
            multiplier = 1;
        if (currentState.equals(State.HALF_SPEED))
            multiplier = 0.5;
        if (currentState.equals(State.H_SCALE_POWER))
            multiplier = l_scale_speed(v_d);
        if (currentState.equals(State.H_SCALE_POWER))
            multiplier = h_scale_speed(v_d);

        fl.setPower( multiplier * (v_d * (Math.sin( (netTheta)  + Math.PI / 4) )) - z );
        fr.setPower( multiplier * (v_d * (Math.sin( (netTheta) + Math.PI / 4) )) + z );
        bl.setPower( multiplier * (v_d * (Math.cos( (netTheta) + Math.PI / 4) )) - z );
        br.setPower( multiplier * (v_d * (Math.cos( (netTheta) + Math.PI / 4) )) + z );
    }
    private double h_scale_speed(double v_d) {
        return Range.clip(0.0308 * Math.exp(4.3891 * v_d), 0.05, 1);
    }
    public void checkState(){
        if (opMode_iterative.gamepad1.left_stick_button && opMode_iterative.gamepad1.a) {
            opMode_iterative.telemetry.addLine("HALF SPEED MODE");
            currentState = State.HALF_SPEED;
            multiplier = 0.5;
        }
        if (opMode_iterative.gamepad1.left_stick_button && opMode_iterative.gamepad1.b) {
            opMode_iterative.telemetry.addLine("H_SCALE");
            currentState = State.H_SCALE_POWER;
        }
        if (opMode_iterative.gamepad1.left_stick_button && opMode_iterative.gamepad1.x) {
            opMode_iterative.telemetry.addLine("L_SCALE");
            currentState = State.L_SCALE_POWER;
        }
        if (opMode_iterative.gamepad1.left_stick_button && opMode_iterative.gamepad1.y) {
            opMode_iterative.telemetry.addLine("FULL SPEED");
            currentState = State.FULL_SPEED;
            multiplier = 1;
        }
    }
    public void moveTelop2 ( double x, double y, double z){

        fr.setPower(multiplier * Range.clip(y - x - z, -1, 1));
        fl.setPower(multiplier * Range.clip(y + x + z, -1, 1));
        br.setPower(multiplier * Range.clip(y + x - z, -1, 1));
        bl.setPower(multiplier * Range.clip(y - x + z, -1, 1));

    }
    public double l_scale_speed(double input){
        // 0.3253ln(x) + 0.9069
        return Range.clip(0.3243 * Math.log(input) + 0.9069, minPower, maxPower);
    }

}