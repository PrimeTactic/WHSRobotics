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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraCalibration;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.io.File;
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

@Autonomous(name="blueautonomousShoot2BeaconCenter", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousShootBeaconCenterBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor collectorMotor;
    private DcMotor shooterMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor frontMotor;
    private DcMotor backMotor;

    private double powerAdjustment = 4;// power will be divided by this factor

    static final double     COUNTS_PER_MOTOR_REV_TETRIX = 1440;
    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: 1440 TETRIX Motor Encoder, 1120 AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP 6.5cm(motor gear)/3.3cm(wheel gear)
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference andymark omniwheel(4in)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    public final static Scalar blueLow = new Scalar(108, 0, 220);//108,0,220
    public final static Scalar blueHigh = new Scalar(178, 255, 255);//178,255,255

    public int BEACON_NOT_VISIBLE = 0;
    public int BEACON_RED_BLUE = 1;
    public int BEACON_BLUE_RED = 2;
    public int BEACON_ALL_BLUE = 3;
    public int BEACON_NO_BLUE = 4;

    VuforiaLocalizer localizer;

    private long debugSleep = 400; // sleep time in milliseconds for debugging

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotors();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdZOZEv/////AAAAGaqoHQLSt0TJrP23HiGwU8Mdqqts2/HFrn5wbhUmoa7zV76jXAPFEM1CQj+Ij86PNScilK4zkkGt6ckTicVLukBfaaw4+tb35Iq8Q7hDlVTgzszgoen+Pa9Um9yT7J6n6yac3PJbb66yYZggOuY4XWSOwBZwAaTIM++2rHVjzHvTsSrltICpyc1g8/UGrXmu78cSIoFV/AwmZrj+ZtKalyeHwKvLfvl/U0VuxDb/4EqjLinRrbragfSo42aCWp+TfDSqIncXn9bmwL6teZqW4LsC/Lvjldbg4tP8ZEK0UEm9stBcmNwi+51o32BHWaoN6g5+1Oy+C2B86O9BopLwTxWgJy3dfbI2foomi0aPb4ou";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = parameters.cameraMonitorFeedback.AXES;
        this.localizer = ClassFactory.createVuforiaLocalizer(parameters);

        if(Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true)){
            Debug("setframeformat to rgb5658");
        }

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = this.localizer.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        VuforiaTrackable legosTrackable = beacons.get(2);
        VuforiaTrackable wheelsTrackable = beacons.get(0);

        VuforiaTrackableDefaultListener legosListener =(VuforiaTrackableDefaultListener) legosTrackable.getListener();
        VuforiaTrackableDefaultListener wheelsListener =(VuforiaTrackableDefaultListener) wheelsTrackable.getListener();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //DELETE test code
        //testMotor(0, 12); // front motor
        //testMotor(1, 12); // right motor
        //testMotor(2, 12); // back motor
        //testMotor(3, 12); // left motor
        //END test code

        beacons.activate();

       int i = encoderDrive(.4, 10,10);

        //get in position to shoot, unwind launcher, then shoot two balls
        shooterDrive(.1, .25); //unwind launcher
        shooterDrive(.1, .9); //shoot first ball
        shooterDrive(.1, .9); //shoot second ball

        i = encoderDrive(1,18,18);

        driveWithEncoderLateral((15*3.14)/3, -(15*3.14)/3);

        i = encoderDrive(1, 30, 30);

        // drive right until it finds the first vision target
        while (opModeIsActive() && wheelsListener.getRawPose() == null){
            frontMotor.setPower(-.2);
            backMotor.setPower(-.2);
        }
        frontMotor.setPower(0);
        backMotor.setPower(0);

        pressBeacon(wheelsTrackable);

        // move to the second vision target on the right which is 48 inches away
        encoderLateralDrive(.5, -48, -48);

        pressBeacon(legosTrackable);


    }

    public void pressBeacon(VuforiaTrackable visionTarget) throws InterruptedException {
        // approach the beacon
        while (opModeIsActive() && !hasReachedBeacon(visionTarget)) {
            approachBeacon(visionTarget);
        }
        Debug("get picture");

        localizer.setFrameQueueCapacity(6);
        Debug("get frame capacity");
        VuforiaLocalizer.CloseableFrame frame = localizer.getFrameQueue().take();
        Debug("frame .take");
        Image img = getImageFromFrame(frame, PIXEL_FORMAT.RGB565);
        Debug("get image from frame");
        CameraCalibration camcal = localizer.getCameraCalibration();
        Debug("camcal");
        VuforiaTrackableDefaultListener visionListener =(VuforiaTrackableDefaultListener) visionTarget.getListener();
        Debug("vision listener");

        int config = getBeaconConfig(img, visionListener, camcal);
        Debug("config:" + config);

        if(config == BEACON_RED_BLUE){
            redOnLeft();
        }else if(config == BEACON_BLUE_RED){
            redOnRight();
        }else if(config == BEACON_ALL_BLUE){
            //press any of the buttons
            redOnRight();
        }
    }

    public Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int pixelFormat){

        long numImgs = frame.getNumImages();

        int counter = 1;

        for(int i = 0; i < numImgs; i++){
            counter++;

            int imgFormat = frame.getImage(i).getFormat();

            if(imgFormat == pixelFormat){
                return frame.getImage(i);
            }
        }
        return null;
    }


    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camcal){


        OpenGLMatrix pose = beacon.getRawPose();

        ByteBuffer pixels = img.getPixels();

        if(pose != null && img != null && pixels != null){
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);

            //calculating pixel coordinates of beacon corners
            float [][] corners = new float[4][2];

            corners[0] = Tool.projectPoint(camcal, rawPose, new Vec3F(-127, 276, 0)).getData(); // top left corner
            corners[1] = Tool.projectPoint(camcal, rawPose, new Vec3F(127, 276, 0)).getData(); // top right corner
            corners[2] = Tool.projectPoint(camcal, rawPose, new Vec3F(127, 92, 0)).getData(); // bottom right corner
            corners[3] = Tool.projectPoint(camcal, rawPose, new Vec3F(-127, 92, 0)).getData(); // bottom left corner

            //start using opencv
            //getting camera image
            int imageWidth = img.getWidth();
            int imageHeight = img.getHeight();

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

            width = (x + width > crop.cols())? crop.cols() - x : width;
            height = (y + height > crop.rows())? crop.rows() - y : height;

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

            //get filtered mask
            //if pixel is within acceptable blue-beacon-color range, it's changed to white
            //otherwise it turns black
            Mat mask = new Mat();

            Core.inRange(cropped, blueLow, blueHigh, mask);

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
        }
        return BEACON_NOT_VISIBLE;
    }

    private void writeBitmap(Bitmap pic, String fileName) throws IOException {
        // Assume block needs to be inside a Try/Catch block.
        String path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES).toString();// Environment.getExternalStorageDirectory().toString();

        OutputStream fOut = null;

        File file = new File(path, fileName); // the File to save , append increasing numeric counter to prevent files from getting overwritten.
        fOut = new FileOutputStream(file);

        pic.compress(Bitmap.CompressFormat.PNG, 85, fOut); // saving the Bitmap to a file compressed as a PNG with 85% compression rate
        fOut.flush(); // Not really required
        fOut.close(); // do not forget to close the stream
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
                    sleep(5000);
                }

                if (isBlackPixel(value) || isWhitePixel(value)){
                    filtered.put(row, col, 0, 255, 0);
                    continue;
                }

                totalNumPixels++;
                if (blueValue > minBlueValue) {
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

    private void turnOffMotors(){

        this.frontMotor.setPower(0);
        this.backMotor.setPower(0);
        this.rightMotor.setPower(0);
        this.leftMotor.setPower(0);

    }

    private void initMotors(){

        /*this.leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("rightMotor");
        this.frontMotor = hardwareMap.dcMotor.get("frontMotor");
        this.backMotor = hardwareMap.dcMotor.get("backMotor");*/
        this.leftMotor  = hardwareMap.dcMotor.get("rightMotor");
        this.rightMotor = hardwareMap.dcMotor.get("leftMotor");
        this.frontMotor = hardwareMap.dcMotor.get("backMotor");
        this.backMotor = hardwareMap.dcMotor.get("frontMotor");
        this.shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        this.collectorMotor = hardwareMap.dcMotor.get("collectorMotor");

        /*this.rightMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        this.leftMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        this.backMotor.setDirection(DcMotor.Direction.FORWARD);
        this.frontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        this.collectorMotor.setDirection(DcMotor.Direction.REVERSE);*/

        this.rightMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        this.leftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        this.backMotor.setDirection(DcMotor.Direction.REVERSE);
        this.frontMotor.setDirection(DcMotor.Direction.FORWARD);
        this.shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        this.collectorMotor.setDirection(DcMotor.Direction.REVERSE);

        turnOffMotors();
    }

    //method for encoder to go certain distance in inches
    private void shooterDrive(double power, double percent){

        //converts the distance to int since settargetposition only accepts int values
        // also uses a formula so that when you type 2 in the robot goes 2 in
        int shooterCounts = (int)Math.round(percent * COUNTS_PER_MOTOR_REV_TETRIX);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor.setTargetPosition(shooterCounts);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        shooterMotor.setPower(power);

        while(opModeIsActive() && shooterMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target distance",  "shooter %7d", shooterCounts);
            telemetry.addData("Current distance",  "shooter %7d", shooterMotor.getCurrentPosition());
            telemetry.update();
        }

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor.setPower(0);
    }

    private void lineUpWithBeacon(VuforiaTrackable beac){
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

        if(pose != null){
            VectorF translation = pose.getTranslation();

            telemetry.addData(beac.getName() + "-Translation", translation);

            double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

            telemetry.addData(beac.getName() + "-degrees", degreesToTurn);

            //find out which array value is x, y, and z
            float x = translation.get(0);
            float y = translation.get(1);
            float z = translation.get(2);

            centerx(beac);
            centerz(beac);

            telemetry.addData(beac.getName() + "x", x);
            telemetry.addData(beac.getName() + "y", y);
            telemetry.addData(beac.getName() + "z", z);

        }
    }

    private boolean hasReachedBeacon(VuforiaTrackable beac) {

        OpenGLMatrix pose;
        VectorF translation;
        while(((VuforiaTrackableDefaultListener) beac.getListener()).getPose() == null) {
        }
        pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

        translation = pose.getTranslation();
        float x = translation.get(0);
        float y = translation.get(1);
        float z = translation.get(2);

        boolean reachedX = false;
        boolean reachedZ = false;

        if(x <= 20 && x >= -20) {
            reachedX = true;
        }

        if(z >= -400){
            reachedZ = true;
        }

        return reachedX && reachedZ;
    }

    private void approachBeacon(VuforiaTrackable beac) {
        OpenGLMatrix pose;
        VectorF translation;

        while ((pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose()) != null) {
            translation = pose.getTranslation();
            float x = translation.get(0);
            float y = translation.get(1);
            float z = translation.get(2);

            if(x <= 25 && x >= -25) {
                // reached the x position
                frontMotor.setPower(0);
                backMotor.setPower(0);
            }else {
                // has not reached x position
                if(x > 100) {
                    frontMotor.setPower(.3);
                    backMotor.setPower(.3);
                }else if(x < -100){
                    frontMotor.setPower(-.3);
                    backMotor.setPower(-.3);
                }else if(x < -25){
                    frontMotor.setPower(-.15);
                    backMotor.setPower(-.15);
                }else if(x > 25){
                    frontMotor.setPower(.2);
                    backMotor.setPower(.2);
                }
            }

            if(z >= -400) {
                // reached the z position
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }else {
                // has not reached the z position
                if(z < -650) {
                    leftMotor.setPower(.4);
                    rightMotor.setPower(.4);
                }else if(z < -500) {
                    leftMotor.setPower(.3);
                    rightMotor.setPower(.3);
                }else {
                    leftMotor.setPower(.2);
                    rightMotor.setPower(.2);
                }
            }
        }
    }

    private void centerx(VuforiaTrackable beac){
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
        if(pose != null){
            VectorF translation = pose.getTranslation();
            float x = translation.get(0);

            while(x <= 15 && x >= -15){
                translation = pose.getTranslation();

                x = translation.get(0);

                if(x > 100) {
                    frontMotor.setPower(.4);
                    backMotor.setPower(.4);
                }else if(x < -100){
                    frontMotor.setPower(-.4);
                    backMotor.setPower(-.4);
                }else if(x < -15){
                    frontMotor.setPower(-.2);
                    backMotor.setPower(-.2);
                }else if(x > 15){
                    frontMotor.setPower(.2);
                    backMotor.setPower(.2);
                }
                frontMotor.setPower(0);
                backMotor.setPower(0);
            }
        }
    }

    private void centerz(VuforiaTrackable beac){
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
        if(pose != null){

            VectorF translation = pose.getTranslation();
            float z = translation.get(2);

            while (z <= -400){
                pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                translation = pose.getTranslation();
                z = translation.get(2);

                if(z < -650) {
                    leftMotor.setPower(.4);
                    rightMotor.setPower(.4);
                }else if(z < -500) {
                    leftMotor.setPower(.3);
                    rightMotor.setPower(.3);
                }else if(z < -400) {
                    leftMotor.setPower(.2);
                    rightMotor.setPower(.2);
                }
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

        }
    }

    private void redOnRight(){
        // distances are in inches
        double xDistance = 0;
        double zDistance = 6;

        //encoderLateralDrive(.3, xDistance, xDistance);
        encoderDrive(.5, zDistance, zDistance);
        encoderDrive(.5, -zDistance, -zDistance);
        //encoderLateralDrive(.3, -7, -7);
    }

    private void redOnLeft(){
        // distances are in inches
        int digit = 0;
        double xDistance = 4;
        double zDistance = 6;

        encoderLateralDrive(.3, -xDistance, -xDistance);
        digit = encoderDrive(.5, zDistance, zDistance);
        digit = encoderDrive(.5, -zDistance, -zDistance);
        encoderLateralDrive(.3, xDistance, xDistance);
    }

    private void testMotor(int motor, double targetDistance) {
        DcMotor driveMotor;
        String motorName;

        switch (motor){
            case 0:
                driveMotor = frontMotor;
                motorName = "Front";
                break;
            case 1:
                driveMotor = rightMotor;
                motorName = "Right";
                break;
            case 2:
                driveMotor = backMotor;
                motorName = "Back";
                break;
            case 3:
                driveMotor = leftMotor;
                motorName = "Left";
                break;
            default:
                driveMotor = null;
                motorName = "None";
        }

        telemetry.addData("Target motor", motorName + ":" + motor);
        telemetry.update();
        sleep(2000);

        int distance = (int)Math.round(targetDistance * COUNTS_PER_INCH);

        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveMotor.setTargetPosition(distance);

        driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        driveMotor.setPower(.2);

        while(opModeIsActive() && driveMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target motor", motorName + ":" + motor);
            telemetry.addData("Target distance", "%.2f", targetDistance);
            telemetry.addData("Current distance", "drive motor %7d", driveMotor.getCurrentPosition());
            telemetry.addData("Current distance",  "left %7d : right %7d",
                    leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.addData("Current distance",  "front %7d : back %7d",
                    frontMotor.getCurrentPosition(), backMotor.getCurrentPosition());
            telemetry.update();
        }

        turnOffMotors();
        sleep(10000);
    }

    //method for encoder to go certain distance in inches
    private int encoderDrive(double power, double distanceLeftDouble , double distanceRightDouble){
        //converts the distance to int since settargetposition only accepts int values
        // also uses a formula so that when you type 2 in the robot goes 2 in
        int distanceLeftInt = (int)Math.round(distanceLeftDouble * COUNTS_PER_INCH);
        int distanceRightInt = (int)Math.round(distanceRightDouble * COUNTS_PER_INCH);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(distanceLeftInt);
        rightMotor.setTargetPosition(distanceRightInt);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        int withinRange = 30;

        //while(opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()){
        while(opModeIsActive() && rightMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target distance",  "left %7d : right %7d", distanceLeftInt,  distanceRightInt);
            telemetry.addData("Current distance",  "left %7d : right %7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();
            if(rightMotor.getCurrentPosition() >= distanceRightInt - withinRange
                    || rightMotor.getCurrentPosition() <= distanceRightInt + withinRange){
                turnOffMotors();
                return 0;
            }
        }

        turnOffMotors();
        return 0;

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderLateralDrive(double power, double distanceFrontDouble , double distanceBackDouble){

        //converts the distance to int since settargetposition only accepts int values
        // also uses a formula so that when you type 2 in the robot goes 2 in
        int distanceFrontInt = (int)Math.round(distanceFrontDouble * COUNTS_PER_INCH);
        int distanceBackInt = (int)Math.round(distanceBackDouble * COUNTS_PER_INCH);

        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontMotor.setTargetPosition(distanceFrontInt);
        backMotor.setTargetPosition(distanceBackInt);

        frontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        frontMotor.setPower(power);
        backMotor.setPower(power);

        int withinRange = 30;

        while(opModeIsActive() && frontMotor.isBusy() && backMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target distance",  "front %7d : back %7d", distanceFrontInt,  distanceBackInt);
            telemetry.addData("Current distance",  "front %7d : back %7d", frontMotor.getCurrentPosition(), backMotor.getCurrentPosition());
            telemetry.update();

            if(frontMotor.getCurrentPosition() >= distanceFrontInt - withinRange
                    || frontMotor.getCurrentPosition() <= distanceFrontInt + withinRange
                    || backMotor.getCurrentPosition() >= distanceBackInt - withinRange
                    || backMotor.getCurrentPosition() <= distanceBackInt + withinRange){
                turnOffMotors();
                return;
            }
        }

        turnOffMotors();

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveWithEncoder(double distanceLeft, double distanceRight){
        distanceLeft = distanceLeft * COUNTS_PER_INCH;
        distanceRight = distanceRight * COUNTS_PER_INCH;

        double distanceLeftStart = .2 * distanceLeft;
        double distanceRightStart = .2 * distanceRight;

        double distanceLeftMiddle = .6 * distanceLeft;
        double distanceRightMiddle = .6 * distanceRight;

        double distanceLeftEnd = .2 * distanceLeft;
        double distanceRightEnd = .2 * distanceRight;

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPostionLeft = leftMotor.getCurrentPosition();
        double currentPostionRight = rightMotor.getCurrentPosition();

        leftMotor.setPower(.2);
        rightMotor.setPower(.2);

        while (leftMotor.getCurrentPosition() < currentPostionLeft + distanceLeftStart && rightMotor.getCurrentPosition() < currentPostionRight + distanceRightStart){
            idle();
        }

        currentPostionLeft = leftMotor.getCurrentPosition();
        currentPostionRight = rightMotor.getCurrentPosition();

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);

        while(leftMotor.getCurrentPosition() < currentPostionLeft + distanceLeftMiddle && rightMotor.getCurrentPosition() < currentPostionRight + distanceRightMiddle){
            idle();
        }

        currentPostionLeft = leftMotor.getCurrentPosition();
        currentPostionRight = rightMotor.getCurrentPosition();

        leftMotor.setPower(.2);
        rightMotor.setPower(.2);

        while(leftMotor.getCurrentPosition() < currentPostionLeft + distanceLeftEnd && rightMotor.getCurrentPosition() < currentPostionRight + distanceRightEnd){
            idle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    private void driveWithEncoderLateral(double distanceForward, double distanceBack){
        distanceForward = distanceForward * COUNTS_PER_INCH;
        distanceBack = distanceBack * COUNTS_PER_INCH;

        double distanceForwardStart = .2 * distanceForward;
        double distanceBackStart = .2 * distanceBack;

        double distanceForwardMiddle = .6 * distanceForward;
        double distanceBackMiddle = .6 * distanceBack;

        double distanceForwardEnd = .2 * distanceForward;
        double distanceBackEnd = .2 * distanceBack;

        frontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPostionForward = frontMotor.getCurrentPosition();
        double currentPostionBack = backMotor.getCurrentPosition();

        frontMotor.setPower(-.2);
        backMotor.setPower(.2);

        while (backMotor.getCurrentPosition() < currentPostionBack + distanceBackStart){
            idle();
        }

        currentPostionForward = frontMotor.getCurrentPosition();
        currentPostionBack = backMotor.getCurrentPosition();

        frontMotor.setPower(-.5);
        backMotor.setPower(.5);

        while(backMotor.getCurrentPosition() < currentPostionBack + distanceBackMiddle){
            idle();
        }

        currentPostionForward = frontMotor.getCurrentPosition();
        currentPostionBack = backMotor.getCurrentPosition();

        frontMotor.setPower(-.2);
        backMotor.setPower(.2);

        while(backMotor.getCurrentPosition() < currentPostionBack + distanceBackEnd){
            idle();
        }

        frontMotor.setPower(0);
        backMotor.setPower(0);
    }
}
