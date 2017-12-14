package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="TeleOpV5", group="Iterative Opmode")

public class TeleOpV5 extends OpMode
{
    // Declare OpMode members.

    //the timer
    private ElapsedTime runtime = new ElapsedTime();
    //wheel motors
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    //claw lift motor
    private DcMotor liftMotor = null;
    //claw servos
    private Servo topLeftServo = null;
    private Servo bottomLeftServo = null;
    private Servo topRightServo = null;
    private Servo bottomRightServo = null;
    //jewel arm servo
    private Servo arm = null;
    //limit switches
    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;
    //speed mode tracker
    private double speedCoef = 1.0;
    //tracks the stick button for the speedmodes
    private boolean buttonTracker = false;
    //the gyroscope
    private BNO055IMU imu = null;
    //variable to get the gyroscope
    private Orientation angles = null;
    //thing to help the user adjust 'forward' when the opmode is running
    public static double angleAdjust;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //the motor variables
        frontLeftMotor  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        backLeftMotor  = hardwareMap.dcMotor.get("back_left_drive");
        backRightMotor = hardwareMap.dcMotor.get("back_right_drive");
        //the lift motors and servos
        liftMotor = hardwareMap.dcMotor.get("grab_lift");
        topLeftServo = hardwareMap.servo.get("top_left");
        bottomLeftServo = hardwareMap.servo.get("bottom_left");
        topRightServo = hardwareMap.servo.get("top_right");
        bottomRightServo = hardwareMap.servo.get("bottom_right");
        //the jewel arm
        arm = hardwareMap.servo.get("back_servo");
        //the lift limit switches
        lowerLimit = hardwareMap.digitalChannel.get("lower_limit");
        upperLimit = hardwareMap.digitalChannel.get("upper_limit");
        // Most robots need the motor on one side to be reversed to drive forward
        //setting the motor directions
        //left wheels go in reverse, right wheels go forward
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        //lift motor gets reversed
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        //left servos get reversed, right servos go forward
        topLeftServo.setDirection(Servo.Direction.REVERSE);
        bottomLeftServo.setDirection(Servo.Direction.REVERSE);
        topRightServo.setDirection(Servo.Direction.FORWARD);
        bottomRightServo.setDirection(Servo.Direction.FORWARD);
        //jewel arm goes forward
        arm.setDirection(Servo.Direction.FORWARD);

        //Getting the inertial measurement unit
        //setting the parameters for the imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //the imu's orientation
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    //not used right now
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;
        double liftPower = 0;

        //toggles between full speed and slow speed
        //driver clicks one of the joysticks to change modes
        //change the number that is not 1.0 to change the speed of slow speed
        if (!buttonTracker && (gamepad1.right_stick_button || gamepad1.left_stick_button))
        {
            speedCoef = (speedCoef == 1.0) ? 0.5 : 1.0;
        }
        buttonTracker = gamepad1.left_stick_button || gamepad1.right_stick_button;

        //finds distance from center of gamepad to current joystick position
        double power = limit(0, 1, Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));
        double target_angle;

        //gets the orientation from the imu
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //driver can press 'a' in order to 'reset' the direction the robot considers to be the front
        if (gamepad1.a)
        {
            angleAdjust = angles.firstAngle;
        }


        //determine angle the driver wants to move in using left stick
        if (gamepad1.left_stick_x == 0)
        {
            if (-gamepad1.left_stick_y >= 0)
            {
                target_angle = 90;
            }
            else
            {
                target_angle = -90;
            }
        }
        else
        {
            target_angle = Math.atan(-gamepad1.left_stick_y / Math.abs(gamepad1.left_stick_x)) * 180 / Math.PI;
        }

        //adjusting the angle system so that 0 degrees is in the front in order to align with the imu's system
        //also allows for moving to the left, as the tangent funtion has the range -90 < y < 90
        if (gamepad1.left_stick_x > 0)
        {
            target_angle -= 90;
        }
        else
        {
            target_angle = -target_angle + 90;
        }

        //does the math to adjust the front
        target_angle -= angles.firstAngle - angleAdjust;

        //calculates the necessary power for the top left and back right wheels
        double power1 = -Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 45 / 180);
        //calculates the necessary power for the top right and back left wheels
        double power2 = -Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 135 / 180);

        //makes sure that both function are within the domain -1 <= x <= 1, while keeping the signs and ratios the same
        power1 = (Math.abs(power1) > Math.abs(power2)) ? Math.abs(power1) / power1 : power1 / Math.abs(power2);
        power2 = (Math.abs(power1) > Math.abs(power2)) ? power2 / Math.abs(power1) : Math.abs(power2) / power2;

        //sets the variables for the motors to the right power, using the numbers for the direction, speed, and speed mode
        frontLeftPower = limit(-1, 1, power1 * power * speedCoef);
        backRightPower = limit(-1, 1, power1 * power * speedCoef);
        frontRightPower = limit(-1, 1, power2 * power * speedCoef);
        backLeftPower = limit(-1, 1, power2 * power * speedCoef);

        //rotates the robot using the driver's right stick
        if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2)
        {
            frontLeftPower += gamepad1.right_stick_x * speedCoef;
            backLeftPower += gamepad1.right_stick_x * speedCoef;
            frontRightPower += -gamepad1.right_stick_x * speedCoef;
            backRightPower += -gamepad1.right_stick_x * speedCoef;
        }

        //makes sure the motor power are in the domain -1 <= x <= 1, keeping signs and ratios the same
        double maxPower;
        if (frontLeftPower > frontRightPower && frontLeftPower > backLeftPower && frontLeftPower > backRightPower)
        {
            maxPower = Math.abs(frontLeftPower);
        }
        else if (frontRightPower > backLeftPower && frontRightPower > backRightPower)
        {
            maxPower = Math.abs(frontRightPower);
        }
        else if (backLeftPower > backRightPower)
        {
            maxPower = Math.abs(backLeftPower);
        }
        else
        {
            maxPower = Math.abs(backRightPower);
        }

        if (maxPower >= 1)
        {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        //moving the lift motor, using the claw operator's left stick and the limit swithes
        if (gamepad2.left_stick_y < 0 && upperLimit.getState() || gamepad2.left_stick_y > 0 && lowerLimit.getState())
        {
            liftPower = -gamepad2.left_stick_y;
        }

        //opens and closes the claw using the right trigger for the upper claw, and the left trigger for the lower claw
        topLeftServo.setPosition(0.85 - gamepad2.right_trigger / (1.5385)); //between 0.85 and 0.2
        topRightServo.setPosition(0.8 - gamepad2.right_trigger / (1.6667)); //between 0.8 and 0.2
        bottomLeftServo.setPosition(0.7 - gamepad2.left_trigger /  (1.8182)); //between 0.7 and 0.15
        bottomRightServo.setPosition(0.75 - gamepad2.left_trigger / (1.6667)); //between 0.75 and 0.15

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        liftMotor.setPower(liftPower);

        //the jewel knocker, unnecessary for teleop
        arm.setPosition(gamepad1.right_trigger / 2 + 0.3);

        // Show the elapsed game time, speed mode, and claw positions
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Speed: ", (speedCoef == 1.0) ? "Fast" : "Slow");
        telemetry.addData("Top Claw: ", (topLeftServo.getPosition() > 0.4) ? "Open" : "Closed");
        telemetry.addData("Bottom Claw: ", (bottomLeftServo.getPosition() > 0.4) ? "Open" : "Closed");
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public double limit(double min, double max, double num)
    {
        return (num < min) ? min : ((num > max) ? max : num);
    }
}
