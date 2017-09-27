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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="controlledDriveShootCollectTest", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class controlledDriveShootCollectTest extends LinearOpMode {

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
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: 1440 TETRIX Motor Encoder, 1120 AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.97 ;     // This is < 1.0 if geared UP 6.5cm(motor gear)/3.3cm(wheel gear)
    static final double     WHEEL_DIAMETER_INCHES   = 1.0 ;     // For figuring circumference andymark omniwheel(4in)
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        collectorMotor  = hardwareMap.dcMotor.get("collectorMotor");
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        leftMotor  = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        frontMotor = hardwareMap.dcMotor.get("frontMotor");
        backMotor = hardwareMap.dcMotor.get("backMotor");




        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        collectorMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontMotor.setDirection(DcMotor.Direction.FORWARD);
        backMotor.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //boolean decides if the collector should be running or not
        Boolean collectorStatus = false;
        int collectorButtonCounter = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Button A", gamepad1.a);
            telemetry.addData("Button B", gamepad1.b);
            telemetry.addData("Collector Counter", collectorButtonCounter);
            telemetry.addData("right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("right_stick_y", gamepad1.right_stick_y);
            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            if(gamepad1.atRest()) {
                turnOffMotors();
            }
            else if(gamepad1.right_stick_y !=0 || gamepad1.right_stick_x != 0){
                driveMotors(-gamepad1.right_stick_x, gamepad1.right_stick_y);
            }
            else if (gamepad1.left_stick_x != 0) {
                turn(gamepad1.left_stick_x);
            }

            // pressing this button once will turn on the collector and pressing it a second time will stop the collector
            if(gamepad1.b) {
                collectorButtonCounter++;
            }

            //pressing this button will prompt the shoot command
            if(gamepad1.a){
                shooterDrive(.1, .9);
            }

            if (collectorButtonCounter > 1500) {
                collectorStatus = !collectorStatus;
                collectorButtonCounter = 0;
            }

            //if the boolean is false, the collector is off, if the boolean is odd, the collector turns on
            if(collectorStatus == true){
                collectorMotor.setPower(1);
            }
            else if(collectorStatus == false) {
                collectorMotor.setPower(0);
            }
        }
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

    private void driveMotors(double x, double y) {
        leftMotor.setPower(y);
        rightMotor.setPower(y);
        frontMotor.setPower(x);
        backMotor.setPower(x);
    }

    private void turn(double power){
        double turningSpeed = power/powerAdjustment;
        frontMotor.setPower(turningSpeed);
        backMotor.setPower(-turningSpeed);
        rightMotor.setPower(-turningSpeed);
        leftMotor.setPower(turningSpeed);
    }

    private void turnOffMotors(){
        frontMotor.setPower(0);
        backMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

}

