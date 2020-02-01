package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import android.graphics.Bitmap;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static android.graphics.Color.RED;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;


/** The Bitmap Vision Class is used to retrieve the position of both Skystones relative to
 * the robot's view of the stone array during init for the Loading Zone side auto and later in the
 * auto for the Building Zone side auto.
 *
 * @author Anish Pandya
 * @version 1.0
 * @since 2019-9-9

 */
public class BitmapVision {
    private static final double LEFT_THRESHOLD_RATIO = 0.445;
    private static final double RIGHT_THRESHOLD_RATIO = 0.723;
    private VuforiaLocalizer vuforia;
    private LinearOpMode opMode;
    private int[] skyPos;
    private final String vuforiaKey = "AU5V2Zr/////AAABmTyLGvLADkQMj7KO5SybwIBFMDGOh6UbhHbOcPIzf2AUeG8hSvpZ5sin2YHPK6iJHHKAs4lTpdmIdZs4VIjEDLjrz3QPFJxSitRYPZlcWZT8MfAyjfz70VVCEUk+mDCiKjZu4JcwI8EO2kcpNnnNfJAkCleZG4/Oa78vCvp11A5D+NT0dQ0fUn23VqzCRnV4lyjGT4wJWZDkmNUac4eW/oDDaEwH02UGQr98rxR1ASe3GAYDubIHMQWwbwleRGj7GhvGCCDwrOznwTLPc/0AjpDckArTEAZtRQUDDp4oTOXiQjXkDJuqXxigfnAu6hghc2/rxkAhr+oATZRZ633m77d1oRGeDuNGy8yFEvhvLY4B";

    private final int aspectRatio = 2;
    // Threshold Values for cutting the pic into only three stones in order to loop through less
    // pixels.
    private final int X_MIN_THRESHOLD = 90 / aspectRatio;
    private final int X_MAX_THRESHOLD = 1200 / aspectRatio;
    private final int Y_MIN_THRESHOLD = 435 / aspectRatio;
    private final int Y_MAX_THRESHOLD = 720 / aspectRatio;

    // Threshold value for the X position of each stone (depends on orientation of webcam/scanning
    // position
    private final int LEFT_THRESHOLD = 600 / aspectRatio;
    private final int MIDDLE_THRESHOLD = 1022 / aspectRatio;
    private final int RIGHT_THRESHOLD = 1275 / aspectRatio;

    private final int RED_THRESHOLD = 121;
    private final int BLUE_THRESHOLD = 121;
    private final int GREEN_THRESHOLD = 121;


    public BitmapVision(LinearOpMode opMode){
        this.opMode = opMode;
        skyPos = new int[2];

        // Vuforia Initialization
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "WC");
        parameters.vuforiaLicenseKey = vuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);

        opMode.telemetry.addLine("Vuforia Init Compete");
        opMode.telemetry.update();
    }
    // used last year; need to test this versus getBitmapVuf();
    private Bitmap getBitmap() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
        long numImgs = frame.getNumImages();
        Image pic = frame.getImage(0);
        Bitmap image;
        for (int i = 1; i < numImgs; i++)
        {
            int format = frame.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                pic = frame.getImage(i);
                break;
            }

            else {
                // Replace with functor
                opMode.telemetry.addLine("RGB 565 FORMAT NOT FOUND - NO PIC");
                opMode.telemetry.update();
            }
        }

        if (pic == null)
        {
            opMode.telemetry.addLine("NULL");
            opMode.telemetry.update();
            image = null;
        }
        else
        {
            image = Bitmap.createBitmap(pic.getWidth(), pic.getHeight(), Bitmap.Config.RGB_565);
            image.copyPixelsFromBuffer(pic.getPixels());
            //opMode.telemetry.addLine("Sucessful Bitmap Creation");
            //opMode.telemetry.update();
        }
        frame.close();

        return image;
    }
    private void findSkyPos() throws InterruptedException
    {

        Bitmap image = getBitmap();
        ArrayList<Integer> xVals = new ArrayList<Integer>();

        opMode.telemetry.addData("Picture Width", image.getWidth());
        opMode.telemetry.addData("Picture Height", image.getHeight());
        opMode.telemetry.update();


        for (int colNum = 0; colNum < image.getWidth(); colNum++){
            for (int rowNum = Y_MIN_THRESHOLD; rowNum < Y_MAX_THRESHOLD; rowNum++){
                int pixel = image.getPixel(colNum,rowNum);

                int red = red(pixel);
                int blue = blue(pixel);
                int green = green(pixel);

                if (red <= 30 && green <= 30 && blue <= 30) {
                    xVals.add(colNum);
                    opMode.telemetry.addData("X val", colNum);
                    opMode.telemetry.update();
                }
            }
        }
        double averageX = 0;
        for (int n: xVals)
            averageX += n;

        try {
            averageX /= xVals.size();
        } catch(ArithmeticException E) {
            averageX = 0;
        }
        opMode.telemetry.addData("AVG X", averageX);
        opMode.telemetry.update();

        if (averageX / image.getWidth() < LEFT_THRESHOLD_RATIO) {
            skyPos[0] = 0;

        } else if (averageX / image.getWidth() < RIGHT_THRESHOLD_RATIO){
            skyPos[0] = 1;

        } else {
            skyPos[0] = 2;

        }
    }
    public int[] getSkyPos(){
        try {
            findSkyPos();
        } catch (InterruptedException e) {
            opMode.telemetry.addLine("Error in findSkyStone()");
        }
        return skyPos;
    }

    // Never used- needs to be tested.
    private Bitmap getBitmapVuf() throws InterruptedException {
        return vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
    }
    private boolean isInRange(int r, int b, int g){
        if (r < 100 && b < 100 && g < 100)
            return true;
        return false;
    }




}
