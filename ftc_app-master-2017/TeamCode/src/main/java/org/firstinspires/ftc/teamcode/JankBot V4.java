package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@TeleOp(name="TeleOpV4", group="Iterative Opmode")

public class TeleOpV4 extends OpMode
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
    private double speedCoef = 1.0;
    private boolean buttonTracker = false;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private char direction;
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        topLeftServo.setDirection(Servo.Direction.FORWARD);
        bottomLeftServo.setDirection(Servo.Direction.FORWARD);
        topRightServo.setDirection(Servo.Direction.REVERSE);
        bottomRightServo.setDirection(Servo.Direction.REVERSE);
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
            speedCoef = (speedCoef == 1.0) ? 0.25 : 1.0;
        }
        buttonTracker = gamepad1.left_stick_button || gamepad1.right_stick_button;

        double power = limit(0, 1, speedCoef * Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y));
        double target_angle;


        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Uses left stick to control the left wheels, and the right stick to control the right wheels

        if (gamepad1.left_stick_x == 0)
        {
            if (-gamepad1.left_stick_y > 0)
            {
                target_angle = 0;
            }
            else
            {
                target_angle = 180;
            }
        }
        else
        {
            target_angle = Math.atan(-gamepad1.left_stick_y / Math.abs(gamepad1.left_stick_x)) * 180 / Math.PI;
            if (gamepad1.left_stick_x > 0)
            {
                target_angle -= 90;
            }
            else
            {
                target_angle = -target_angle + 90;
            }
        }

        target_angle -= angles.firstAngle;

        frontLeftPower = limit(-1, 1, (-Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 45 / 180)) * power * speedCoef);
        backRightPower = limit(-1, 1, (-Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 45 / 180)) * power * speedCoef);
        frontRightPower = limit(-1, 1, (-Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 135 / 180)) * power * speedCoef);
        backLeftPower = limit(-1, 1, (-Math.sqrt(2) * Math.sin(Math.PI * target_angle / 180 - Math.PI * 135 / 180)) * power * speedCoef);

        if (gamepad1.right_stick_x > 0.2 || gamepad1.right_stick_x < -0.2)
        {
            frontLeftPower = gamepad1.right_stick_x * speedCoef;
            backLeftPower = gamepad1.right_stick_x * speedCoef;
            frontRightPower = -gamepad1.right_stick_x * speedCoef;
            backRightPower = -gamepad1.right_stick_x * speedCoef;
        }

        if (gamepad2.dpad_up)
        {
            liftPower = 1;
        }
        else if (gamepad2.dpad_down)
        {
            liftPower = -1;
        }
        else
        {
            liftPower = 0;
        }

        if (gamepad2.left_stick_x > 0.2)
        {
            topLeftServo.setPosition(1);
            topRightServo.setPosition(1);
        }
        else if (gamepad2.left_stick_x < -0.2)
        {
            topLeftServo.setPosition(0);
            topRightServo.setPosition(0);
        }

        if (gamepad2.right_stick_x > 0.2)
        {
            bottomLeftServo.setPosition(1);
            bottomRightServo.setPosition(1);
        }
        else if (gamepad2.right_stick_x < -0.2)
        {
            bottomLeftServo.setPosition(0);
            bottomRightServo.setPosition(0);
        }

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        liftMotor.setPower(liftPower);
        arm.setPosition(gamepad1.right_trigger / 2.0);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servo Angles: ", "Top Left: " + topLeftServo.getPosition() + "\nBottom Left: " + bottomLeftServo.getPosition() + "\nTop Right: " + topRightServo.getPosition() + "\nBottom Right: " + bottomRightServo.getPosition());
        telemetry.addData("Speed: ", (speedCoef == 1.0) ? "Fast" : "Slow");
        //telemetry.addData("Angle: ", angles.firstAngle);
        telemetry.addData("Direction: ", target_angle);
        telemetry.addData("Power: ", power);
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
