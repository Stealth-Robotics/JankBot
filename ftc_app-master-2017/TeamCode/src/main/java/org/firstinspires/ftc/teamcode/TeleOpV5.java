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
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor liftMotor = null;
    private Servo topLeftServo = null;
    private Servo bottomLeftServo = null;
    private Servo topRightServo = null;
    private Servo bottomRightServo = null;
    private Servo arm = null;
    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;
    private double speedCoef = 1.0;
    private boolean buttonTracker = false;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private double angleAdjust = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        backLeftMotor  = hardwareMap.dcMotor.get("back_left_drive");
        backRightMotor = hardwareMap.dcMotor.get("back_right_drive");
        liftMotor = hardwareMap.dcMotor.get("grab_lift");
        topLeftServo = hardwareMap.servo.get("top_left");
        bottomLeftServo = hardwareMap.servo.get("bottom_left");
        topRightServo = hardwareMap.servo.get("top_right");
        bottomRightServo = hardwareMap.servo.get("bottom_right");
        arm = hardwareMap.servo.get("back_servo");
        lowerLimit = hardwareMap.digitalChannel.get("lower_limit");
        upperLimit = hardwareMap.digitalChannel.get("upper_limit");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        topLeftServo.setDirection(Servo.Direction.REVERSE);
        bottomLeftServo.setDirection(Servo.Direction.REVERSE);
        topRightServo.setDirection(Servo.Direction.FORWARD);
        bottomRightServo.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);

        //Getting the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;
        double liftPower = 0;

        if (buttonTracker == false && gamepad1.right_stick_button == true || gamepad1.left_stick_button == true)
        {
            speedCoef = (speedCoef == 1.0) ? 0.4 : 1.0;
        }
        buttonTracker = gamepad1.left_stick_button || gamepad1.right_stick_button;

        double power = limit(0, 1, speedCoef * Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));
        double target_angle;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (gamepad1.a)
        {
            angleAdjust = angles.firstAngle;
        }

        // Uses left stick to control the left wheels, and the right stick to control the right wheels

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

        if (gamepad1.left_stick_x > 0)
        {
            target_angle -= 90;
        }
        else
        {
            target_angle = -target_angle + 90;
        }

        target_angle -= angles.firstAngle - angleAdjust;

        if (target_angle > 180)
        {
            target_angle -= 360;
        }
        else if (target_angle < -180)
        {
            target_angle += 360;
        }

        double power1 = -Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 45 / 180);
        double power2 = -Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 135 / 180);

        power1 = (Math.abs(power1) > Math.abs(power2)) ? Math.abs(power1) / power1 : power1 / Math.abs(power2);
        power2 = (Math.abs(power1) > Math.abs(power2)) ? power2 / Math.abs(power1) : Math.abs(power2) / power2;

        frontLeftPower = limit(-1, 1, power1 * power * speedCoef);
        backRightPower = limit(-1, 1, power1 * power * speedCoef);
        frontRightPower = limit(-1, 1, power2 * power * speedCoef);
        backLeftPower = limit(-1, 1, power2 * power * speedCoef);

        if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2)
        {
            frontLeftPower += gamepad1.right_stick_x * speedCoef * speedCoef;
            backLeftPower += gamepad1.right_stick_x * speedCoef * speedCoef;
            frontRightPower += -gamepad1.right_stick_x * speedCoef * speedCoef;
            backRightPower += -gamepad1.right_stick_x * speedCoef * speedCoef;
        }

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

        frontLeftPower /= maxPower;
        backLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backRightPower /= maxPower;

        if (gamepad2.left_stick_y < -0.2 && upperLimit.getState())
        {
            liftPower = 1;
        }
        else if (gamepad2.left_stick_y > 0.2 && lowerLimit.getState())
        {
            liftPower = -1;
        }
        else
        {
            liftPower = 0;
        }

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
        arm.setPosition(gamepad1.right_trigger / 2.7);

        // Show the elapsed game time and wheel power.
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
