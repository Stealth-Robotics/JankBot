package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.text.SimpleDateFormat;
import java.util.Date;

@Disabled
@Autonomous(name="AutoV3RED_RIGHT", group="Red Linear Opmode")

public class AutoV3RED_RIGHT extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private Servo arm = null;
    private Servo topLeftServo = null;
    private Servo bottomLeftServo = null;
    private Servo topRightServo = null;
    private Servo bottomRightServo = null;
    private DcMotor liftMotor = null;

    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackable relicTemplate = null;
    private RelicRecoveryVuMark vuMark = null;

    private ColorSensor sensorColor = null;
    private BNO055IMU imu = null;
    private Orientation angles = null;
    private Position position = null;

    @Override
    public void runOpMode()
    {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        backLeftMotor  = hardwareMap.dcMotor.get("back_left_drive");
        backRightMotor = hardwareMap.dcMotor.get("back_right_drive");
        arm = hardwareMap.servo.get("back_servo");
        topLeftServo = hardwareMap.servo.get("top_left");
        bottomLeftServo = hardwareMap.servo.get("bottom_left");
        topRightServo = hardwareMap.servo.get("top_right");
        bottomRightServo = hardwareMap.servo.get("bottom_right");
        liftMotor = hardwareMap.dcMotor.get("grab_lift");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_range_sensor");

        //Getting the IMU
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        arm.setDirection(Servo.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        topLeftServo.setDirection(Servo.Direction.REVERSE);
        bottomLeftServo.setDirection(Servo.Direction.REVERSE);
        topRightServo.setDirection(Servo.Direction.FORWARD);
        bottomRightServo.setDirection(Servo.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXwhG3X/////AAAAGeyrECmhIEnMtx00hFsdD204jVm8PsOfnpnVu7sU9FQnzbhyt14ohqXYBSOB2xsEM11XgvKUZE5HtTvnoQ7JNknNvg9GtTt9P+LCq/TNS3pam2n8FfmzKypVR5M2PZQ/d9MhR0AfHwJa+UE7G0b8NevUKCya1wd+qwK3k5pTEaI81q6Z4iHVl+u1O0eICRG6bj+M2q36ZKtm/CQzcnmCSQYJhIDQsZL2/78EJiWUtO/GjVrEYoNwNLxCkiln6UQEKO4zWW6TcuwRMgO9f++DI3EDQdp9ads3vxh33/I38KirR/izKL8Wf0/qrqucbxv8B8QWh1tR8/ojvNN12Vc6ib1yyAqYrjz7hJ4jqFGJ2ADY";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        bottomLeftServo.setPosition(0.15);
        bottomRightServo.setPosition(0.15);
        sleep(500);
        liftMotor.setPower(1);
        sleep(700);
        liftMotor.setPower(0);

        arm.setPosition(0.77);
        sleep(500);
        if (sensorColor.red() < sensorColor.blue())
        {
            frontLeftMotor.setPower(0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(0.2);
            backRightMotor.setPower(-0.2);
        }
        else
        {
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(0.2);
        }
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        sleep(300);

        if (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0)
        {
            frontLeftMotor.setPower(-0.2);
            frontRightMotor.setPower(0.2);
            backLeftMotor.setPower(-0.2);
            backRightMotor.setPower(0.2);
        }
        else
        {
            frontLeftMotor.setPower(0.2);
            frontRightMotor.setPower(-0.2);
            backLeftMotor.setPower(0.2);
            backRightMotor.setPower(-0.2);
        }
        arm.setPosition(1);
        while (Math.abs(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) >= 5 && !isStopRequested())
        {
            if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
        }
        telemetry.addData("VuMark: ", vuMark);
        telemetry.update();

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);
        sleep(1200);
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

        switch (vuMark)
        {
            case LEFT:
                sleep(100);
            case CENTER:
                sleep(100);
            case RIGHT:
                sleep(400);
                break;
            default:
                sleep(500);
        }

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);

        while (!(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= -95 || imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= -90) && !isStopRequested())
        {
            if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
        }

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        sleep(300);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        bottomLeftServo.setPosition(0.7);
        bottomRightServo.setPosition(0.75);

        TeleOpV5.angleAdjust = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}
