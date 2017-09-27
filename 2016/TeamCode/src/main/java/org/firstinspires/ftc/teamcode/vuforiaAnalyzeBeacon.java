/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;
import com.vuforia.ar.pl.DebugLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.Arrays;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="autonomousAnalyzeBeacon", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class vuforiaAnalyzeBeacon extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public final static Scalar blueLow = new Scalar(108, 0, 220);//108,0,220
    public final static Scalar blueHigh = new Scalar(178, 255, 255);//178,255,255

    public int BEACON_NOT_VISIBLE = 0;
    public int BEACON_RED_BLUE = 1;
    public int BEACON_BLUE_RED = 2;
    public int BEACON_ALL_BLUE = 3;
    public int BEACON_NO_BLUE = 4;

    VuforiaLocalizer localizer;

    private long debugSleep = 800; // sleep time in milliseconds for debugging

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdZOZEv/////AAAAGaqoHQLSt0TJrP23HiGwU8Mdqqts2/HFrn5wbhUmoa7zV76jXAPFEM1CQj+Ij86PNScilK4zkkGt6ckTicVLukBfaaw4+tb35Iq8Q7hDlVTgzszgoen+Pa9Um9yT7J6n6yac3PJbb66yYZggOuY4XWSOwBZwAaTIM++2rHVjzHvTsSrltICpyc1g8/UGrXmu78cSIoFV/AwmZrj+ZtKalyeHwKvLfvl/U0VuxDb/4EqjLinRrbragfSo42aCWp+TfDSqIncXn9bmwL6teZqW4LsC/Lvjldbg4tP8ZEK0UEm9stBcmNwi+51o32BHWaoN6g5+1Oy+C2B86O9BopLwTxWgJy3dfbI2foomi0aPb4ou";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = parameters.cameraMonitorFeedback.AXES;
        this.localizer = ClassFactory.createVuforiaLocalizer(parameters);

        if(Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true)){
            Debug("setframeformat to rgb5658");
        }else{
            Debug("rgb565 not supported");
        }

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = this.localizer.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        VuforiaTrackableDefaultListener wheels =(VuforiaTrackableDefaultListener) beacons.get(0).getListener();

        /*
        example
        if(config == BEACON_AlL_BLUE){
        drive somewhere
        }
         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        beacons.activate();

        while (opModeIsActive() && wheels.getRawPose() == null){
            idle();
        }

        localizer.setFrameQueueCapacity(6);
        VuforiaLocalizer.CloseableFrame frame = localizer.getFrameQueue().take();

        Image img = getImageFromFrame(frame, PIXEL_FORMAT.RGB565);

        CameraCalibration camcal = localizer.getCameraCalibration();

        int config = getBeaconConfig(img, wheels, camcal);

        telemetry.addData("beacon config", config);
        telemetry.update();

    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){

        long numImgs = frame.getNumImages();

        Debug("num images: " + numImgs);

        int counter = 1;

        for(int i = 0; i < numImgs; i++){

            Debug("for loop" + counter);
            counter++;

            int imgFormat = frame.getImage(i).getFormat();

            Debug("imageformat" + imgFormat);
            if(imgFormat == pixelFormat){
                Debug("return image from frame: " + i);
                return frame.getImage(i);
            }
        }

        Debug("return image from frame: null+++++++++++++++++++++++++++++++");
        return null;
    }


    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camcal){


        OpenGLMatrix pose = beacon.getRawPose();
        /*if(pose == null){
            Debug("pose null");
        }

        //img is null right now with no beacon above it
        if(img == null){
            Debug("img null");
        }*/

        ByteBuffer pixels = img.getPixels();
        /*if(pixels == null){
            Debug("pixels null");
        }*/

        if(pose != null && img != null && pixels != null){
            //Debug("Computing Beacon 0");
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            //Debug("Computing Beacon 1");

            //calculating pixel coordinates of beacon corners
            float [][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camcal, rawPose, new Vec3F(-127, 276, 0)).getData(); // top left corner
            corners[1] = Tool.projectPoint(camcal, rawPose, new Vec3F(127, 276, 0)).getData(); // top right corner
            corners[2] = Tool.projectPoint(camcal, rawPose, new Vec3F(127, 92, 0)).getData(); // bottom right corner
            corners[3] = Tool.projectPoint(camcal, rawPose, new Vec3F(-127, 92, 0)).getData(); // bottom left corner

            //Debug("Computing Beacon 2");

            //start using opencv
            //getting camera image
            int imageWidth = img.getWidth();
            int imageHeight = img.getHeight();
            Debug("Image (width, height): " + imageWidth + ", " + imageHeight);

            Bitmap bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(img.getPixels());

            try {
                writeBitmap(bm, "image6.png");
            } catch (IOException e) {
                Debug(e.getMessage());
                e.printStackTrace();
            }

            //turning pixel coordinates to a proper boundary box
            int matHeight = bm.getHeight();
            int matWidth = bm.getWidth();
            Mat crop = new Mat(matHeight, matWidth, CvType.CV_8UC3);

            Utils.bitmapToMat(bm, crop);

            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));

            //make sure boundary box isn't bigger than image
            x = Math.max(x, 0);
            y = Math.max(y, 0);
            Debug("X: " + x + ", Y: " + y);

            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

            Debug("Width: " + width + ", Height: " + height);

            //cropping boundery box out of camera image
            Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));

            Bitmap croppedBitmap = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, croppedBitmap);
            try {
                writeBitmap(croppedBitmap, "croppedImage6.png");
            } catch (IOException e) {
                Debug(e.getMessage());
                e.printStackTrace();
            }

            //filtering out non-beacon-blue colors in HSV color space
            /*Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);

            Bitmap hsvBitmap = Bitmap.createBitmap(cropped.width(), cropped.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, hsvBitmap);
            try {
                writeBitmap(hsvBitmap, "hsvImage6.png");
            } catch (IOException e) {
                Debug(e.getMessage());
                e.printStackTrace();
            }*/

            //get filtered mask
            //if pixel is within acceptable blue-beacon-color range, it's changed to white
            //otherwise it turns black
            Mat mask = new Mat();

            Core.inRange(cropped, blueLow, blueHigh, mask);

            Debug("after core.inrange");
            Debug("mask width" + mask.width());
            Debug("mask height" + mask.height());
            Debug("mask channels" + mask.channels());

            Bitmap maskBitmap = Bitmap.createBitmap(mask.width(), mask.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(cropped, maskBitmap);
            try {
                writeBitmap(maskBitmap, "mask6.png");
            } catch (IOException e) {
                Debug(e.getMessage());
                e.printStackTrace();
            }

            int state = analyzeBeaconState(cropped);
            Debug("state: " + state);
            return state;

            /*Moments mnts = Imgproc.moments(mask, true);

            Debug("Computing Beacon 5");

            //calculating centroid of the binary mask
            Debug("Centroid x: " + ((mnts.get_m10() / mnts.get_m00())));
            Debug("Centroid y: " + ((mnts.get_m01() / mnts.get_m00())));

            //checking if blue either takes up the majority of the image (beacon all blue)
            //or barely any (beacon all red)
            if(mnts.get_m00() / mask.total() > 0.8) {
                return BEACON_ALL_BLUE;
            } else if (mnts.get_m00() / mask.total() < .1){
                return BEACON_NO_BLUE;
            }

            //fix it team gets a  image rotated 90 degrees
            // if centroid bottom half, blud on left, vice versa if centroid on top
            if(mnts.get_m01() / mnts.get_m00() < cropped.rows() / 2){
                return BEACON_RED_BLUE;
            } else {
                return  BEACON_BLUE_RED;
            }*/
        }

        Debug("Beacon is not visible");
        return BEACON_NOT_VISIBLE;
    }

    private void writeMat(Mat mask){
        //boolean t =  Highgui.imwrite(filename, mIntermediateMat);
    }

    private void writeBitmap(Bitmap pic, String fileName) throws IOException {
        // Assume block needs to be inside a Try/Catch block.
        String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES).toString();// Environment.getExternalStorageDirectory().toString();
        Debug("Storage path: " + path);

        OutputStream fOut = null;

        File file = new File(path, fileName); // the File to save , append increasing numeric counter to prevent files from getting overwritten.
        fOut = new FileOutputStream(file);

        pic.compress(Bitmap.CompressFormat.PNG, 85, fOut); // saving the Bitmap to a file compressed as a PNG with 85% compression rate
        fOut.flush(); // Not really required
        fOut.close(); // do not forget to close the stream

        Debug("File path: " + file.getAbsolutePath());
    }

    private int analyzeBeaconState(Mat cropped){
        int height = cropped.height();
        int width = cropped.width();
        double sum = 0;
        double leftSum = 0;
        int leftSide = height/2;

        double minBlueValue = 120;
        //int totalNumPixels = height * width;
        int totalNumPixels = 0;

        Mat filtered = new Mat(height, width, CvType.CV_8UC3);

        for(int row = 0; row < height; row++){
            for(int col = 0; col < width; col++){
                double[] value = cropped.get(row, col);
                double redValue = value[0];
                double blueValue = value[2];

                if (row == 500 & col == 25)
                {
                    Debug("pixel value: " + value[0] + ", " + value[1] + ", " + value[2]);
                    sleep(5000);
                }

                if (isBlackPixel(value) || isWhitePixel(value)){
                    filtered.put(row, col, 0, 255, 0);
                    continue;
                }

                totalNumPixels++;
                if (blueValue > minBlueValue) {
                    //Debug("value array length " + value.length);
                    //Debug("Value[2] blue component(row: " + row + ",col: " + col + "): " + value[2]);
                    filtered.put(row, col, 0, 0, 255);
                    sum++;
                    if (row > leftSide) {
                        leftSum++;
                    }
                }
                else {
                    filtered.put(row, col, 255, 0, 0);
                }
            }
        }

        Bitmap filteredBitmap = Bitmap.createBitmap(filtered.width(), filtered.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(filtered, filteredBitmap);
        try {
            writeBitmap(filteredBitmap, "filtered6.png");
        } catch (IOException e) {
            Debug(e.getMessage());
            e.printStackTrace();
        }

        Debug("Percent pixels included in analysis: " + (totalNumPixels/(height*width)));
        sleep(5000);

        double percentBlue = sum/totalNumPixels;
        telemetry.addData("percent blue", percentBlue);
        telemetry.addData("sum", sum);
        telemetry.addData("area", totalNumPixels);
        telemetry.update();
        sleep(5000);
        //Debug("percentblue: " + percentBlue);
        //Debug("sum: " + sum);
        //Debug("area: " + (height * width));
        double percentblueLeft = leftSum/totalNumPixels;

        if(percentBlue > .8){
            return BEACON_ALL_BLUE; //blue
        }else if(percentBlue <.2){
            return  BEACON_NO_BLUE; //red
        }else if(percentblueLeft < .2) {
            return BEACON_RED_BLUE;
        }else{
            return BEACON_BLUE_RED;
        }
    }

    private boolean isWhitePixel(double[] pixel){
        double minColorValue = 150;

        if (pixel[0] >= minColorValue
                && pixel[1] >= minColorValue
                && pixel[2] >= minColorValue) {
            return true;
        }
        else{
            return false;
        }
    }

    private boolean isBlackPixel(double[] pixel){
        double maxColorValue = 100;

        if (pixel[0] <= maxColorValue
                && pixel[1] <= maxColorValue
                && pixel[2] <= maxColorValue) {
            return true;
        }
        else{
            return false;
        }
    }

    private void Debug(String message){
        telemetry.addLine(message);
        telemetry.update();
        sleep(debugSleep);
    }


}
