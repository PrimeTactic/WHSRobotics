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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@Autonomous(name="encoderTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousCornerStart extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor frontMotor;
    private DcMotor backMotor;
    private DcMotor shooterMotor;
    private DcMotor collectorMotor;
    static final double     COUNTS_PER_MOTOR_REV_TETRIX = 1440;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: 1440 TETRIX Motor Encoder, 1120 AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP 6.5cm(motor gear)/3.3cm(wheel gear)
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference andymark omniwheel(4in)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode()  {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotors();


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double shooterSpeed = .1;

        encoderDrive(.2, (15*3.14)/4, -(15*3.14)/4 );
        sleep(3000);
        encoderDrive(.2, 12, 12);
        sleep(5000);
        driveWithEncoder(12, 12);

        shooterDrive(shooterSpeed, 1);
        sleep(500);
        shooterDrive(shooterSpeed, 1);

        sleep(5000);

        encoderDrive(.2, 18, 18);

        encoderDrive(.2, (15*3.14)/8, -(15*3.14)/8 );


        //leftMotor.setPower(1);
        //rightMotor.setPower(1);
        //sleep(3000);
        //leftMotor.setPower(0);
        //rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //method for encoder to go certain distance in inches
    private void encoderDrive(double power, double distanceLeftDouble , double distanceRightDouble){

        //converts the distance to int since settargetposition only accepts int values
        // also uses a formula so that when you type 2 in the robot goes 2 in
        int distanceLeftInt = (int)Math.round(distanceLeftDouble * COUNTS_PER_INCH);
        int distanceRightInt = (int)Math.round(distanceRightDouble * COUNTS_PER_INCH);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(distanceLeftInt);
        rightMotor.setTargetPosition(distanceRightInt);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        forward(power);

        while(opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target distance",  "left %7d : right %7d", distanceLeftInt,  distanceRightInt);
            telemetry.addData("Current distance",  "left %7d : right %7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();
        }

        turnOffMotors();

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //method that makes robot go forward or backwards depending on power input
    private void forward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    //method makes robot turn left
    private void turnLeft(int time) throws InterruptedException{
        leftMotor.setPower(-1);
        rightMotor.setPower(1);
        Thread.sleep(time);
    }

    //method makes robot turn right
    private void turnRight(int time) throws InterruptedException{
        leftMotor.setPower(1);
        rightMotor.setPower(-1);
        Thread.sleep(time);
    }

    private void turnOffMotors(){
        this.frontMotor.setPower(0);
        this.backMotor.setPower(0);
        this.rightMotor.setPower(0);
        this.leftMotor.setPower(0);
    }

    private void initMotors(){
        this.leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        this.rightMotor = hardwareMap.dcMotor.get("rightMotor");
        this.frontMotor = hardwareMap.dcMotor.get("frontMotor");
        this.backMotor = hardwareMap.dcMotor.get("backMotor");
        this.shooterMotor = hardwareMap.dcMotor.get("shooterMotor");

        this.leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        this.rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        this.frontMotor.setDirection(DcMotor.Direction.REVERSE);
        this.backMotor.setDirection(DcMotor.Direction.FORWARD);
        this.shooterMotor.setDirection(DcMotor.Direction.FORWARD);

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

        shooterMotor.setPower(0);
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

    //method for encoder to go certain distance in inches
    private void encoderTurn(double power, double distance){

        //converts the distance to int since settargetposition only accepts int values
        // also uses a formula so that when you type 2 in the robot goes 2 in
        int distanceTurn = (int)Math.round(distance * COUNTS_PER_INCH);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(distanceTurn);
        rightMotor.setTargetPosition(-distanceTurn);
        backMotor.setTargetPosition(-distanceTurn);
        rightMotor.setTargetPosition(distanceTurn);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // drive
        backMotor.setPower(power);
        frontMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setPower(power);

        while(opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()){
            // Display it for the driver.
            telemetry.addData("Target distance", distance);
            telemetry.addData("Current distance",  "left %7d : right %7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();
        }

        turnOffMotors();

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
