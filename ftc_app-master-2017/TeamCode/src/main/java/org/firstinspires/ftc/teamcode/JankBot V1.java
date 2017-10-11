package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpV1", group="Iterative Opmode")

public class TeleOpV1 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
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
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        // Uses left stick to control the left wheels, and the right stick to control the right wheels
        if (gamepad1.left_stick_x >= -0.2 && gamepad1.left_stick_x <= 0.2)
        {
            frontLeftPower = 1 * -gamepad1.left_stick_y;
            backLeftPower = 1 * -gamepad1.left_stick_y;
        }
        else if (gamepad1.left_stick_x > 0.2)
        {
            frontLeftPower = 1 * gamepad1.left_stick_x;
            backLeftPower = -1 * gamepad1.left_stick_x;
        }
        else
        {
            frontLeftPower = 1 * gamepad1.left_stick_x;
            backLeftPower = -1 * gamepad1.left_stick_x;
        }

        if (gamepad1.right_stick_x >= -0.2 && gamepad1.right_stick_x <= 0.2)
        {
            frontRightPower = 1 * -gamepad1.right_stick_y;
            backRightPower = 1 * -gamepad1.right_stick_y;
        }
        else if (gamepad1.right_stick_x > 0)
        {
            frontRightPower = -1 * gamepad1.right_stick_x;
            backRightPower = 1 * gamepad1.right_stick_x;
        }
        else
        {
            frontRightPower = -1 * gamepad1.right_stick_x;
            backRightPower = 1 * gamepad1.right_stick_x;
        }

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
        telemetry.addData("Right Stick Y: ", gamepad1.right_stick_y);
        telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
        //telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
