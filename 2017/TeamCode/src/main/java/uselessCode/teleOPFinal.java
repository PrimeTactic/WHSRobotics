/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package uselessCode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

import static java.lang.Math.sqrt;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 * http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf
 * https://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot
 */

// this is a test comment
@TeleOp(name="teleOPFinal", group="Linear Opmode")
@Disabled
public class teleOPFinal extends LinearOpMode {
    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1; // back motor
    private DcMotor motor2; // left motor
    private DcMotor motor3; // right motor
    private Servo armServo;
    private Servo leftClampServo;
    private Servo rightClampServo;
    int cooldown = 1000;
    private boolean isClamped = true;
    private double row1Position = 0.2;
    private double row2Position = 0.4;
    private double row3Position = 0.6;
    private BNO055IMU gyro;
    final private double OPENCLAMPPOSITION = 0;
    final private double CLOSECLAMPPOSITION = .5;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "gyro";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        armServo = hardwareMap.get(Servo.class, "armServo");
        leftClampServo = hardwareMap.get(Servo.class, "leftClampServo");
        rightClampServo = hardwareMap.get(Servo.class, "rightClampServo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        armServo.setDirection(Servo.Direction.FORWARD);
        rightClampServo.setDirection(Servo.Direction.REVERSE);
        leftClampServo.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            // on gamepad movement (controls robot wheel movment)
            if (gamepad1.atRest())  // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            {
                turnOffMotors();
            }
            else
            {
                drive(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                turn(gamepad1.left_stick_x);
            }

            // on button presses (controls arm movement)
            if (gamepad1.a)
            {
                armServo.setPosition(.8);
            }
            else if (gamepad1.b) // middle arm position
            {
                armServo.setPosition(.4);
            }
            else if (gamepad1.y)
            {
                armServo.setPosition(0);
            }
            else if(gamepad1.dpad_down)
            {
                lowerArm(armServo.getPosition());
            }
            else if(gamepad1.dpad_up)
            {
                liftArm(armServo.getPosition());
            }
            else if(gamepad1.left_trigger > .5)
            {
                drive(-1, 0); //drive left
            }
            else if(gamepad1.right_trigger > .5)
            {
                drive(1, 0); //drive right
            }

            if (gamepad1.x && cooldown <= 0) // only allow toggling the claws every 100 loops
            {
                // toggles if the arm is clamping every time x is pressed
                cooldown = 1000; // reset cooldown
                isClamped = !isClamped;
                if (isClamped) {
                    setClampPosition(OPENCLAMPPOSITION);
                }
                else {
                    setClampPosition(CLOSECLAMPPOSITION);
                }
            }
            else if (cooldown > 0) {
                cooldown--;
            }
            else if(gamepad1.dpad_down || gamepad1.dpad_right )
            {
                if(gamepad1.dpad_right)
                {
                    rightClampServo.setPosition(.4);
                    leftClampServo.setPosition(.4);
                }
                else if(gamepad1.dpad_left)
                {
                    closeClamp(rightClampServo.getPosition());
                }
            }

            // Show the elapsed game time and wheel power
            updateTelemetry();
        }
    }

    public void updateTelemetry(){
        //double armServoValue = armServo.getPosition();
        //double leftClampValue = leftClampServo.getPosition();
        //double rightClampValue = rightClampServo.getPosition();
        //boolean aButton = gamepad1.a;
        telemetry.addData("Status", "Run Time   : " + runtime.toString());
        telemetry.addData("Arm servo position   : " , armServo.getPosition());
        telemetry.addData("Right clamp position : " , rightClampServo.getPosition());
        telemetry.addData("left motor power", motor2.getPower());
        telemetry.addData("right motor power", motor3.getPower());
        telemetry.addData("left encoder", motor2.getCurrentPosition());
        telemetry.addData("right encoder", motor3.getCurrentPosition());
        telemetry.addData("front motor power", motor1.getPower());
        telemetry.addData("x value right stick  : " , gamepad1.right_stick_x);
        telemetry.addData("y value right stick  : " , -gamepad1.right_stick_y);
        AngularVelocity v = gyro.getAngularVelocity();
        float v_x = v.xRotationRate;
        telemetry.addData("x rotation rate : " , v_x);

        telemetry.update();
    }

    public double sigmoid(double x) {
        return 1 / (1 + Math.exp(-x));
    }

    //drive method that accepts two values, x and y motion
    public void drive(double x, double y)
    {
        double scale = 1;
        // for precise movement
        //if (gamepad1.right_bumper) {
        //    scale = 0.5;
        //}

        
        final float correctionZoneDegrees = 5;
        AngularVelocity v = gyro.getAngularVelocity();
        float v_x = v.xRotationRate; // positive is clockwise
        double correctionValue = 0;
        if (Math.abs(v_x) > correctionZoneDegrees)
        {
            correctionValue = (double)(v_x / 200.0);
        }
        
        double power1 = scale * x;
        double power2 = (scale * (((-.5) * x) - (sqrt(3)/2) * y)) - correctionValue;
        double power3 = (scale * (((-.5) * x) + (sqrt(3)/2) * y)) - correctionValue;

        
        
        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);

        telemetry.addData("motor 3 power" , power2);
        telemetry.addData("motor 2 power" , power3);
        telemetry.addData("correction value" , correctionValue);

    }

    private void turnOffMotors()
    {
        motor1.setPower(0);
        motor3.setPower(0);
        motor2.setPower(0);
    }

    private void turn(double speed)
    {
        double divisor = 2;
        // of the bumper is being held than make the robot turn slower
        if (gamepad1.left_bumper) {
            divisor = 4;
        }
        motor1.setPower(-speed/divisor);
        motor3.setPower(-speed/divisor);
        motor2.setPower(-speed/divisor);
    }

    private void setClampPosition(double newClampPosition)
    {
        rightClampServo.setPosition(newClampPosition);
        leftClampServo.setPosition(newClampPosition);
    }

    private void closeClamp(double currentPosition)
    {
        double newPosition = currentPosition + .0005;
        rightClampServo.setPosition(newPosition);
        leftClampServo.setPosition(newPosition);
    }

    private  void openClamp(double currentPosition)
    {
        double newPosition = currentPosition - .0005;
        rightClampServo.setPosition(newPosition);
        leftClampServo.setPosition(newPosition);
    }

    private void liftArm(double currentPosition)
    {
        double newPosition = currentPosition - .001;
        armServo.setPosition(newPosition);
    }

    private  void lowerArm(double currentPosition)
    {
        double newPosition = currentPosition + .001;
        armServo.setPosition(newPosition);
    }
    
}
