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
    private boolean encoderStrafe;
    private int loopCount =0;

    private enum State {
        FULL_SPEED,
        REGULAR_SPEED,
        LOW_SPEED,
        H_SCALE_POWER;
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
    boolean changeDpadDown = false;
    boolean changeDpadUp = false;

    // Instance Variables

    private ExpansionHubMotor fl, fr, bl, br;
    private ElapsedTime drivetrainClock;
    public Map<String, Double> sensorVals;
    public double[] encoderVals;
    private double[] powers;
    private PID pidControlller;
    private Sensors sensors;
    private double maxPower = 1;
    private double minPower = 0.05;
    private int multCounter = 1;
    private double[] multipliers = {0.3, 0.55 , 1};
    private String[] multipiersTelemetry = {"LOW POWER", "REGULAR POWER", "HIGH POWER"};
    private double prevEncoder;
    private boolean isMoving;
    //private ExpansionHubEx expansionHub;
    ///private RevBulkData bulkdata;
    private boolean reset;
    private double multiplier;


    public Drivetrain(LinearOpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {


        this.opMode = opMode;
        sensors = new Sensors(this.opMode);
        powers = new double[NUM_MOTORS];
        encoderVals = new double[NUM_MOTORS];

        // Tracks Sensor Vals.
        this.sensorVals = sensorVals;
        drivetrainClock = new ElapsedTime();
        currentState = State.FULL_SPEED;

        fl = (ExpansionHubMotor) this.opMode.hardwareMap.dcMotor.get("fl");
        fr = (ExpansionHubMotor) this.opMode.hardwareMap.dcMotor.get("fr");
        bl = (ExpansionHubMotor) this.opMode.hardwareMap.dcMotor.get("bl");
        br = (ExpansionHubMotor) this.opMode.hardwareMap.dcMotor.get("br");

        // encoderVals[FRONT_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fl);
        // encoderVals[FRONT_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fr);
        // encoderVals[BACK_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(bl);
        // encoderVals[BACK_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(br);


        sensorVals.put("Current Encoder", getEncoderAverage(Math.PI / 2));
        sensorVals.put("Current Angle", sensors.getFirstAngle());
        sensorVals.put("Timestamp", 0.0);
        encoderStrafe = false;

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

        resetEncoders();

        opMode.telemetry.addLine("Drivetrain Init Completed");
        opMode.telemetry.update();
    }

    public Drivetrain(OpMode opMode, ElapsedTime timer, Map<String, Double> sensorVals) throws InterruptedException {
        this.opMode_iterative = opMode;
        sensors = new Sensors(this.opMode_iterative);
        double[] powers = new double[NUM_MOTORS];
        encoderVals = new double[4];

        opMode_iterative.telemetry.addLine("Drivetrain update");
        opMode_iterative.telemetry.update();

        // Tracks Sensor Vals.
        //this.sensorVals = sensorVals;
        drivetrainClock = new ElapsedTime();

        currentState = State.FULL_SPEED;

        fl = (ExpansionHubMotor) this.opMode_iterative.hardwareMap.dcMotor.get("fl");
        fr = (ExpansionHubMotor) this.opMode_iterative.hardwareMap.dcMotor.get("fr");
        bl = (ExpansionHubMotor) this.opMode_iterative.hardwareMap.dcMotor.get("bl");
        br = (ExpansionHubMotor) this.opMode_iterative.hardwareMap.dcMotor.get("br");


        //encoderVals[FRONT_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fl);
        //encoderVals[FRONT_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(fr);
        //encoderVals[BACK_LEFT] = expansionHub.getBulkInputData().getMotorCurrentPosition(bl);
        //encoderVals[BACK_RIGHT] = expansionHub.getBulkInputData().getMotorCurrentPosition(br);


        //sensorVals.put("Current Encoder", getEncoderAverage());
        //sensorVals.put("Timestamp", 0.0);

        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        encoderStrafe = false;

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opMode_iterative.telemetry.addLine("Drivetrain Complete");
        opMode_iterative.telemetry.update();
/*
        pidControlller = new PID();
        pidControlller.setReset(true);
        multiplier = 1;
        */
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

    public void resetEncoders() {
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

    public void getEncoders() {
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

    public void setFl(ExpansionHubMotor fl) {
        this.fl = fl;
    }

    public DcMotor getFr() {
        return fr;
    }

    public void setFr(ExpansionHubMotor fr) {
        this.fr = fr;
    }

    public DcMotor getBl() {
        return bl;
    }

    public void setBl(ExpansionHubMotor bl) {
        this.bl = bl;
    }

    public DcMotor getBr() {
        return br;
    }

    public void movveStrafeDS(){

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
    public double getEncoderAverage(double angle) {
        //getEncoders();
        double encoderAverage = 0;
        int counter = 0;
        for (int i = 0; i < encoderVals.length; i++) {
            if (encoderVals[i] == 0) {
                counter++;
            }
            encoderAverage += encoderVals[i];
        }
        try {
            return (encoderAverage / (4 - counter));
        } catch (ArithmeticException E) {
            return 0;
        }
    }

    public double getEncoderAverageES(double angle) {
        getEncoders();
        double encoderAverage = 0;
        if (angle == (Math.PI / 2) || angle == (3 * Math.PI / 2)) {
            int counter = 0;
            for (int i = 0; i < encoderVals.length; i++) {
                if (encoderVals[i] == 0) {
                    counter++;
                }
                encoderAverage += encoderVals[i];
            }
            if (counter == 4)
                return 0;
            try {
                return (encoderAverage / (4 - counter));
            } catch (ArithmeticException E) {
                return 0;
            }
        } else if (angle == 0) {
            return (encoderVals[FRONT_RIGHT] + encoderVals[BACK_LEFT]) * 0.5;
        } else if (angle > 0 && angle < Math.PI / 2) {
            return (encoderVals[FRONT_LEFT] + encoderVals[BACK_RIGHT]) * 0.5;
        } else if (angle > Math.PI / 2 && angle < Math.PI) {
            return (encoderVals[FRONT_RIGHT] + encoderVals[BACK_LEFT]) * 0.5;
        } else if (angle == Math.PI) {
            return (encoderVals[FRONT_LEFT] + encoderVals[BACK_RIGHT]) * 0.5;
        } else if (angle > Math.PI && angle < 3 * Math.PI / 2)
            return (encoderVals[FRONT_LEFT] + encoderVals[BACK_RIGHT]) * 0.5;
        else
            return (encoderVals[FRONT_LEFT] + encoderVals[BACK_LEFT]) * 0.5;
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
        double currentPos = getEncoderAverage(Math.PI / 2);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < timeout && currentPos < inchesToEncoder(distance)) { //timer loop to stop motors after time reaches timeout or if destination is reached
            getEncoders();
            currentPos = getEncoderAverage(Math.PI / 2);
            setAllMotors(power);
        }
        setAllMotors(0);
    }
    public void correctHeading(double timeout) {

        double currAngle = sensors.getFirstAngle();
        double target = 0;

        pidControlller.setReset(true);
        pidControlller.setCoeffs(0.5/currAngle, 0,0.2/currAngle);

        ElapsedTime t_i = new ElapsedTime();

        opMode.telemetry.addData("Curr Angle", currAngle);
        opMode.telemetry.update();

        while (!inBounds(currAngle, -1,1) &&  t_i.seconds() < timeout && opMode.opModeIsActive()) {
            turn(pidControlller.iteration(currAngle, t_i.seconds()), !((Math.signum(currAngle)) > 0));
            currAngle = sensors.getFirstAngle();
            sensorVals.put("Current Angle", currAngle);

            opMode.telemetry.addData("INSIDE LOOP: t_i", t_i);
            opMode.telemetry.addData("INSIDE LOOP: error", currAngle);
            opMode.telemetry.addData("LOOP SPEED", loopCount/t_i.seconds());
            opMode.telemetry.update();
            loopCount++;

        }
        setAllMotors(0);
    }
    public double correctHeading2(double p, double d, ElapsedTime t_i, double target, double currAngle) {

        pidControlller.setReset(true);
        pidControlller.setCoeffs(p, 0, d);
        double currAngleError = Math.toDegrees(target) - currAngle;
        if (!inBounds(currAngleError, -1,1)) {
            currAngleError = Math.toDegrees(target) - currAngle;
            return pidControlller.iteration(currAngleError, t_i.seconds()) * Math.signum(currAngle) * -1;
        }
        return 0;
    }

    private boolean inBounds(double num, double lowBound, double highBound) {
        if (num >= lowBound && num <= highBound)
            return true;
        return false;
    }

    private void setPowers(double v_d, double v_theta, double angle){

        powers[FRONT_LEFT] = (v_d * Math.sin(angle) + v_d * Math.cos(angle) - v_theta);
        powers[FRONT_RIGHT] = (v_d * Math.sin(angle) - v_d * Math.cos(angle) + v_theta);
        powers[BACK_LEFT] = (v_d * Math.sin(angle) - v_d * Math.cos(angle) - v_theta);
        powers[BACK_RIGHT] = (v_d * Math.sin(angle) + v_d * Math.cos(angle) + v_theta);
    }
    private void scaleMotors(){
        double maxPower = powers[0];

        for (int i = 1; i < powers.length; i++)
            maxPower = Math.max(Math.abs(maxPower), Math.abs(powers[i]));

        if (maxPower > 1) {
            powers[FRONT_LEFT] /= maxPower;
            powers[FRONT_RIGHT] /= maxPower;
            powers[BACK_LEFT] /= maxPower;
            powers[BACK_RIGHT] /= maxPower;
        }

    }


    /**
     * Move or strafe in the cardinal directions with the option to turn while doing so
     *
     * @param v_d     - Desired Velocity
     * @param v_theta - Desired Rotational Velocity
     * @param angle   - Desired Angle
     * @param distance - Desired distance in encoder ticks
     * @param timeout - maximum time for the movement
     * @param percentTolerance  - acceptable percentage of error.
     */
    public void move(double v_d, double v_theta, double angle, double distance, double timeout, double percentTolerance) {
        resetEncoders();
        ElapsedTime timer = new ElapsedTime();
        double initPos = getEncoderAverageES(Math.toRadians(sensors.getFirstAngle()));
        // Bounds calculated by the percentTolerance
        double low_bound = distance * (1 - percentTolerance);
        double high_bound = distance * (1 + percentTolerance);
        // Loop Condition
        timer.reset();
        boolean loopCondition = Math.abs(initPos - getEncoderAverageES(angle)) < low_bound
                && (timer.seconds() < timeout)
                && opMode.opModeIsActive();
        // Calculate motor powers
        setPowers(v_d, v_theta, angle);

        // Scale Motors while conserving ratios between them
        scaleMotors();

        timer.reset();
        while (loopCondition) {
            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);

            loopCondition = Math.abs(initPos - getEncoderAverageES(angle)) < low_bound
                    && (timer.seconds() < timeout)
                    && opMode.opModeIsActive();
        }
        setAllMotors(0);
    }
    //fs
    public void moveStrafeY(double v_d, double v_theta, double angle, double distance, double timeout, double percentTolerance) {
        resetEncoders();
        ElapsedTime timer = new ElapsedTime();
        double initPos = sensorVals.get("Y");

        // Bounds calculated by the percentTolerance
        double low_bound = distance * (1 - percentTolerance);
        double high_bound = distance * (1 + percentTolerance);

        // Loop Condition
        boolean loopCondition = !inBounds(initPos - sensorVals.get("Y"), low_bound, high_bound )
                && (timer.seconds() < timeout) && opMode.opModeIsActive();

        // Calculate motor powers
        setPowers(v_d, v_theta, angle);

        // Scale Motors while conserving ratios between them
        scaleMotors();

        timer.reset();
        while (loopCondition) {

            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);

            opMode.telemetry.addData("Y:", sensorVals.get("Y"));
            opMode.telemetry.update();

            loopCondition = !inBounds(initPos - sensorVals.get("Y"), low_bound, high_bound )
                    && (timer.seconds() < timeout) && opMode.opModeIsActive();
        }
        setAllMotors(0);
    }
    public void moveStrafeX(double v_d, double v_theta, double angle, double distance, double timeout, double percentTolerance) {
        resetEncoders();
        ElapsedTime timer = new ElapsedTime();
        double initPos = sensorVals.get("X");

        // Bounds calculated by the percentTolerance
        double low_bound = distance * (1 - percentTolerance);
        double high_bound = distance * (1 + percentTolerance);

        // Loop Condition
        boolean loopCondition = !inBounds(sensorVals.get("X"), low_bound, high_bound )
                && (timer.seconds() < timeout) && opMode.opModeIsActive();

        // Calculate motor powers
        setPowers(v_d, v_theta, angle);

        // Scale Motors while conserving ratios between them
        scaleMotors();

        timer.reset();
        while (loopCondition) {

            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);

            loopCondition = !inBounds(sensorVals.get("X"), low_bound, high_bound )
                    && (timer.seconds() < timeout) && opMode.opModeIsActive();
        }
        setAllMotors(0);
    }
    public void moveToHeading(double v_d, double distance, double timeout, double angle, double percentTolerance) {
        PID rotational = new PID();
        PID linear = new PID();

        resetEncoders();
        double prevErrorCheck_d;
        double prevErrorCheck_a;
        double d_low_bound = distance * (1 - percentTolerance);
        double d_high_bound = distance * (1 + percentTolerance);
        double a_low_bound = angle * (1 - percentTolerance);
        double a_high_bound = angle * (1 + percentTolerance);

        int countA = 0;
        int countD = 0;

        rotational.setTarget(angle);
        linear.setTarget(distance);

        // Set Motor Powers for set time
        ElapsedTime timer = new ElapsedTime();

        double currAngle = sensors.getFirstAngle();
        double v_theta;
        double initPos = getEncoderAverageES(Math.toRadians(currAngle));


        timer.reset();
        boolean loopCondition = rotational.checkState(a_low_bound, a_high_bound)
                && linear.checkState(d_low_bound, d_high_bound)
                && (timer.seconds() < timeout)
                && opMode.opModeIsActive();

        while (loopCondition) {
            currAngle = sensors.getFirstAngle();
            sensorVals.put("Current Angle", currAngle);

            // correctHeading2((0.5 / currAngle), (0.2 / currAngle), timer, angle, currAngle);
            v_theta = rotational.iteration(angle - currAngle, timer.seconds() );
            if (sensorVals.get("Current Drivetrain Encoder Average") != null) {
                v_d = linear.iteration(sensorVals.get("Current Drivetrain Encoder Average"), timer.seconds());
            } else{
                getEncoders();
                v_d = linear.iteration(getEncoderAverage(currAngle), timer.seconds());
            }

            setPowers(v_d, v_theta, angle);
            scaleMotors();


            fl.setPower(powers[FRONT_LEFT]);
            fr.setPower(powers[FRONT_RIGHT]);
            bl.setPower(powers[BACK_LEFT]);
            br.setPower(powers[BACK_RIGHT]);

            prevErrorCheck_a = rotational.getPreviousError();
            prevErrorCheck_d = linear.getPreviousError();


            loopCondition = rotational.checkState(a_low_bound, a_high_bound)
                    && linear.checkState(d_low_bound, d_high_bound)
                    && (timer.seconds() < timeout)
                    && opMode.opModeIsActive()
                    && countD < 6
                    && countA < 6;


             if (prevErrorCheck_a == rotational.getPreviousError())
                 countA++;
             if (prevErrorCheck_d == linear.getPreviousError())
                 countD++;

        }
        setAllMotors(0);
    }
    //TODO
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
    public void turnGyro(double power, double target, boolean right, double timeout) {
        double angle = sensors.getFirstAngle(); //TODO: Replacement for getting gyro angles
        ElapsedTime timer = new ElapsedTime();
        while (Math.abs(angle - sensors.getFirstAngle()) < target && right && timer.seconds() < timeout) {
            turn(power, true);
        }
        while (!right && Math.abs(angle - sensors.getFirstAngle()) < target&& timer.seconds() < timeout) {
            turn(power, false);
        }
    }

    /**
     * Strafe Encoder
     */
    public void setEncoderStrafe(boolean encoderStrafe) {
        this.encoderStrafe = encoderStrafe;
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
    public void turnPID(double dTheta, double k_p, double k_i, double k_d, double timeout, boolean right) throws InterruptedException {
        pidControlller.setReset(true);
        pidControlller.setCoeffs(k_p, k_i, k_d);

        double theta_i = sensors.getFirstAngle();
        sensorVals.put("Current Angle", theta_i);

        double target = dTheta + theta_i;

        ElapsedTime t_i = new ElapsedTime();

        pidControlller.setT_i(0);
        pidControlller.setTarget(target);

        double error = target;
        opMode.telemetry.addData("t_i", t_i);
        opMode.telemetry.addData("error", error);
        opMode.telemetry.update();

        Thread.sleep(3000);

        while (Math.abs(error) > 0.3 && t_i.seconds() < timeout && opMode.opModeIsActive()) {
            sensorVals.put("Current Angle", sensors.getFirstAngle());

            error = target - Math.abs(sensorVals.get("Current Angle"));

            double power = Range.clip(pidControlller.iteration(error, t_i.seconds()) + 0.2,-1,1);

            turn(power, !right);

            opMode.telemetry.addData("ERROR", error);
            opMode.telemetry.addData("POWER", power);

            opMode.telemetry.update();
        }

    }
    /*
        public void turnPID(double dTheta, double k_p, double k_i, double k_d, double timeout, boolean right) {
        pidControlller.setReset(true);
        pidControlller.setCoeffs(k_p, k_i, k_d);



        double theta_i = sensors.getFirstAngle();
        double target = dTheta + theta_i;

        ElapsedTime t_i = new ElapsedTime();

        pidControlller.setT_i(t_i.seconds());
        pidControlller.setTarget(target);

        double error = target;
        opMode.telemetry.addData("t_i", t_i);
        opMode.telemetry.addData("error", error);
        opMode.telemetry.update();
        while (Math.abs(error) > 0.3 && t_i.seconds() < timeout && opMode.opModeIsActive()) {
            if (error > 0)
                turn(pidControlller.iteration(error, t_i.seconds()), !right);
            error = target - sensors.getFirstAngle();

            opMode.telemetry.addData("INSIDE LOOP: t_i", t_i);
            opMode.telemetry.addData("INSIDE LOOP: error", error);
            opMode.telemetry.update();
        }

    }
     */

    /*
     monitorEncoders();
     - Monitors encoders during every bulk read.
     -  change in change in encoder ticks > 0.
     - Value of Encoder ticks != 0.
  */
    public void monitorEncoders(double encoderAverage) {


    }

    /**
     * @param -       inputted number to angleWrap in degrees
     * @param radians - boolean on whether to output in radians or degrees
     * @return
     */
    public double angleWrap(double input, boolean radians) {
        if (input > 180) {
            input -= 360;
        }
        if (radians) {
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
        double v_d = Math.hypot(x, y) * Math.signum(x) * Math.signum(y);
        double netTheta = Math.atan2(x, y);
        if (v_d < 0.05)
            v_d = 0;
        if (v_d > 0.95)
            v_d = 1;
        toggleSpeed();

        if (currentState.equals(State.FULL_SPEED))
            multiplier = 1;
        if (currentState.equals(State.REGULAR_SPEED))
            multiplier = 0.5;
        if (currentState.equals(State.LOW_SPEED))
            multiplier = .25;
        if (currentState.equals(State.H_SCALE_POWER))
            multiplier = h_scale_speed(v_d);

        fl.setPower(multiplier * (v_d * (Math.sin((netTheta) + Math.PI / 4))) - z);
        fr.setPower(multiplier * (v_d * (Math.sin((netTheta) + Math.PI / 4))) + z);
        bl.setPower(multiplier * (v_d * (Math.cos((netTheta) + Math.PI / 4))) - z);
        br.setPower(multiplier * (v_d * (Math.cos((netTheta) + Math.PI / 4))) + z);
    }

    private double h_scale_speed(double v_d) {
        return Range.clip(0.0308 * Math.exp(4.3891 * v_d), 0.05, 1);
    }

    public void toggleSpeed() {

        if ((opMode_iterative.gamepad1.dpad_down && !changeDpadDown) && multCounter > 0) {
            multCounter--;
        }
        else if ((opMode_iterative.gamepad1.dpad_up && !changeDpadUp) && multCounter < 2) {
            multCounter++;
        }
        multiplier = multipliers[multCounter];
        opMode_iterative.telemetry.addLine(multipiersTelemetry[multCounter]);

        if (opMode_iterative.gamepad1.left_stick_button && opMode_iterative.gamepad1.b) {
            opMode_iterative.telemetry.addLine("H_SCALE");
            currentState = State.H_SCALE_POWER;
        }
        changeDpadDown = opMode_iterative.gamepad1.dpad_down;
        changeDpadUp = opMode_iterative.gamepad1.dpad_up;
        opMode_iterative.telemetry.update();
    }

    public void moveTelop2(double x, double y, double z) {
        double[] powers = new double[NUM_MOTORS];
        powers[FRONT_RIGHT] = y - x - z;
        powers[FRONT_LEFT] = y + x + z;
        powers[BACK_RIGHT] = y + x - z;
        powers[BACK_LEFT] = y - x + z;

        double maxPower = powers[0];
        for (int i = 1; i < NUM_MOTORS; i++) {
            maxPower = Math.max(maxPower, powers[i]);
        }
        if (maxPower > 1) {
            powers[FRONT_RIGHT] /= maxPower;
            powers[FRONT_LEFT] /= maxPower;
            powers[BACK_LEFT] /= maxPower;
            powers[BACK_RIGHT] /= maxPower;
        }
        fl.setPower(multiplier * powers[FRONT_LEFT]);
        fr.setPower(multiplier * powers[FRONT_RIGHT]);
        br.setPower(multiplier * powers[BACK_RIGHT]);
        bl.setPower(multiplier * powers[BACK_LEFT]);
    }

    public void testEncodersGyro(){
        if (reset){
            resetEncoders();
        }
        opMode.telemetry.addData("Encoder FL:", fl.getCurrentPosition() );
        opMode.telemetry.addData("Encoder FR:", fr.getCurrentPosition() );
        opMode.telemetry.addData("Encoder BL:", bl.getCurrentPosition() );
        opMode.telemetry.addData("Encoder BR:", br.getCurrentPosition() );
        opMode.telemetry.addData("Gyro Angle:", sensors.getFirstAngle());
        opMode.telemetry.update();

    }
}