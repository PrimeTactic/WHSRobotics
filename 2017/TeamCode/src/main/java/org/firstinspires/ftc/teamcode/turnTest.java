package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.sqrt;

@TeleOp(name="turnTest", group="Linear Opmode")

public class turnTest extends LinearOpMode {
    //teleOPFinal

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1; // back motor
    private DcMotor motor2; // left motor
    private DcMotor motor3; // right motor
    private Servo armServo;
    private Servo leftClampServo;
    private Servo rightClampServo;
    final double armLiftChange = 0.0015;
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
        double elapsedTime;

        //Always True
        while (opModeIsActive() ) {


            if (gamepad1.a) {
                turn(.5);
                sleep(500);
                turnOffMotors();
            } else if (gamepad1.b) // middle arm position
            {
                turn(.5);
                sleep(1000);
                turnOffMotors();
            } else if (gamepad1.y) {
                turn(.5);
                sleep(1500);
                turnOffMotors();
            } else if (gamepad1.x) {
                turn(.5);
                sleep(2000);
                turnOffMotors();
            }

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
        double newPosition = currentPosition - armLiftChange;
        armServo.setPosition(newPosition);
    }

    private  void lowerArm(double currentPosition)
    {
        double newPosition = currentPosition + armLiftChange;
        armServo.setPosition(newPosition);
    }

}

