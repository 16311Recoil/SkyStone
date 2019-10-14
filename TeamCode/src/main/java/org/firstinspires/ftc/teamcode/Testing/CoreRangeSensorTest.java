package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.TreeMap;

public class CoreRangeSensorTest extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;
    TreeMap<String, Double> sensorVal;
    BulkRead bulkReader;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        initialize();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("Distance Sensor Value", sensorVal.get("Analog 1"));
            telemetry.update();
        }
    }
    public void hardwareMap(){
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
    }
    public void initialize(){
        sensorVal = new TreeMap<String, Double>();
        bulkReader = new BulkRead(this);
        bulkReader.run();
    }
    public class BulkRead implements Runnable{
        ExpansionHubEx expansionHubEx;
        RevBulkData bulkReader;
        LinearOpMode opMode;

        public BulkRead(LinearOpMode opMode){
            this.opMode = opMode;
            expansionHubEx = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
            bulkReader = expansionHubEx.getBulkInputData();
            sensorVal.put("Analog 1", ((double) bulkReader.getAnalogInputValue(1)));
        }


        @Override
        public void run() {
            while(!opMode.isStopRequested()){
                bulkReader = expansionHubEx.getBulkInputData();
                sensorVal.put("Analog 1", ((double) bulkReader.getAnalogInputValue(1)));
            }
        }
    }

}
