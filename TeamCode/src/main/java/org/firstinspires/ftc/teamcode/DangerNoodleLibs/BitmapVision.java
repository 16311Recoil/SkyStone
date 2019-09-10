package org.firstinspires.ftc.teamcode.DangerNoodleLibs;

import android.graphics.Bitmap;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
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
    private VuforiaLocalizer vuforia;
    private LinearOpMode opMode;
    private int[] skyPos;
    private final String vuforiaKey = "";

    // Threshold Values for cutting the pic into only three stones in order to loop through less
    // pixels.
    private final int X_MIN_THRESHOLD = 0;
    private final int X_MAX_THRESHOLD = 0;
    private final int Y_MIN_THRESHOLD = 0;
    private final int Y_MAX_THRESHOLD = 0;

    // Threshold value for the X position of each stone (depends on orientation of webcam/scanning
    // position
    private final int LEFT_THRESHOLD = 0;
    private final int MIDDLE_THRESHOLD = 0;
    private final int RIGHT_THRESHOLD = 0;

    private final int RED_THRESHOLD = 0;
    private final int BLUE_THRESHOLD = 0;
    private final int GREEN_THRESHOLD = 0;


    public BitmapVision(LinearOpMode opMode){
        this.opMode = opMode;
        skyPos = new int[2];

        // Vuforia Initialization
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.vuforiaLicenseKey = vuforiaKey;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
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
            opMode.telemetry.addLine("Sucessful Bitmap Creation");
            opMode.telemetry.update();
        }
        frame.close();

        return image;
    }
    private void findSkyPos() throws InterruptedException
    {
        Bitmap image = getBitmap();
        ArrayList<Integer> xVals = new ArrayList<Integer>();
        for (int x = X_MIN_THRESHOLD; x < X_MAX_THRESHOLD; x+=2) {
            for (int y = Y_MIN_THRESHOLD; y < Y_MAX_THRESHOLD; x+=2) {
                int pixel = image.getPixel(x,y);

                int red = red(pixel);
                int blue = blue(pixel);
                int green = green(pixel);

                if (red > RED_THRESHOLD && blue > BLUE_THRESHOLD && green > GREEN_THRESHOLD)
                    xVals.add(x);
            }
        }
        int averageX = 0;
        for (int n: xVals)
            averageX += n;
        try {
            averageX /= xVals.size();
        } catch(ArithmeticException E) {
            averageX = 0;
        }
        if (averageX > LEFT_THRESHOLD && averageX < MIDDLE_THRESHOLD) {
            skyPos[0] = 0;
            skyPos[1] = 3;
        } else if (averageX > MIDDLE_THRESHOLD && averageX < RIGHT_THRESHOLD){
            skyPos[0] = 1;
            skyPos[1] = 4;
        } else {
            skyPos[0] = 2;
            skyPos[1] = 5;
        }
    }
    // Never used- needs to be tested.
    private Bitmap getBitmapVuf() throws InterruptedException {
        return vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
    }




}
