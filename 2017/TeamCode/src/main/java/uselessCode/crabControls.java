package uselessCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;

@TeleOp(name="Crab_Controls", group="Linear Opmode")
@Disabled
public class crabControls extends LinearOpMode {
    //teleOPFinal

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1; // back motor
    private DcMotor motor2; // left motor
    private DcMotor motor3; // right motor
    private Servo armServo;
    private Servo leftClampServo;
    private Servo rightClampServo;
    final double cooldown = 0.5;
    private boolean isClamped = true;
    private double row1Position = 0.2;
    private double row2Position = 0.4;
    private double row3Position = 0.6;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        armServo = hardwareMap.get(Servo.class, "armServo");
        leftClampServo = hardwareMap.get(Servo.class, "leftClampServo");
        rightClampServo = hardwareMap.get(Servo.class, "rightClampServo");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        armServo.setDirection(Servo.Direction.FORWARD);
        rightClampServo.setDirection(Servo.Direction.REVERSE);
        leftClampServo.setDirection(Servo.Direction.FORWARD);

        //At Start
        waitForStart();
        runtime.reset();
        double clawActiveTimeInSec = 0;
        boolean clawActive = false;


        //Always True
        while (opModeIsActive()) {
            // on gamepad movement (controls robot wheel movment)
            if (gamepad1.atRest()) {
                turnOffMotors();
            } else {
                driveStraight(-gamepad1.right_stick_y);
                turn(gamepad1.left_stick_x);
            }

            //Arm Height
            if (gamepad1.a) {
                armServo.setPosition(.8);
            } else if (gamepad1.b) // middle arm position
            {
                armServo.setPosition(.4);
            } else if (gamepad1.y) {
                armServo.setPosition(0);
            } else if (gamepad1.dpad_down) {
                lowerArm(armServo.getPosition());
            } else if (gamepad1.dpad_up) {
                liftArm(armServo.getPosition());
            }

            //Left to Right Movement
            if (gamepad1.left_trigger > .5) {
                drive(-1, 0); //drive left
            } else if (gamepad1.right_trigger > .5) {
                drive(1, 0); //drive right
            }

            // The x button can only be pressed when the claw is not active.
            if (gamepad1.x && !clawActive) // only allow toggling the claws every 100 loops
            {
                clawActive = true; // set the claw to active
                clawActiveTimeInSec = runtime.time(); // get the time we set the claw to active

                // toggles if the arm is clamping every time x is pressed
                isClamped = !isClamped;
                if (isClamped) {
                    rightClampServo.setPosition(0);
                    leftClampServo.setPosition(0);
                } else {
                    rightClampServo.setPosition(.5);
                    leftClampServo.setPosition(.5);
                }
            }
            else if (clawActive)
            {
                double currentTimeInSec = runtime.time();
                if ((currentTimeInSec - clawActiveTimeInSec) > cooldown)
                {
                    clawActive = false;
                }
            }

            if (gamepad1.dpad_down || gamepad1.dpad_right)
            {
                if (gamepad1.dpad_right)
                {
                    rightClampServo.setPosition(.4);
                    leftClampServo.setPosition(.4);
                }
                else if (gamepad1.dpad_left)
                {
                    closeClamp(rightClampServo.getPosition());
                }
            }

            // Show the elapsed game time and wheel power

            //updateTelemetry();
        }
    }

    public void updateTelemetry(){

        telemetry.addData("Status", "Run Time   : " + runtime.toString());
        telemetry.addData("Arm servo position   : " , armServo.getPosition());
        telemetry.addData("Right clamp position : " , rightClampServo.getPosition());
        telemetry.addData("x value right stick  : " , gamepad1.right_stick_x);
        telemetry.addData("y value right stick  : " , -gamepad1.right_stick_y);
        telemetry.addData("left motor power", motor2.getPower());
        telemetry.addData("right motor power", motor3.getPower());
        telemetry.addData("back motor power", motor1.getPower());
        telemetry.addData("left encoder", motor2.getCurrentPosition());
        telemetry.addData("right encoder", motor3.getCurrentPosition());
        telemetry.addData("back encoder", motor1.getCurrentPosition());
        telemetry.update();
    }
    public double sigmoid(double x) {
        return 1 / (1 + Math.exp(-x));
    }

    //drive method that accepts two values, x and y motion

    public void drive(double x, double y)
    {
        double scale1 = 1.0;
        double scale2 = 1.0;
        double scale3 = 1.0;
        double power1 = scale1 * x;
        double power2 = (scale2 * (((-.5) * x) - (sqrt(3)/2) * y)) ;
        double power3 = (scale3 * (((-.5) * x) + (sqrt(3)/2) * y)) ;

        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
    }

    public  void driveStraight(double y)
    {
        if (y > 0)
        {
            y = 1;
        }
        else if (y < 0)
        {
            y = -1;
        }

        motor2.setPower(-y);
        motor3.setPower(y);
    }

    public  void driveBackward(double y)
    {
        motor2.setPower(-1);
        motor3.setPower(1);
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

