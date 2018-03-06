package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class AutoTestPixy2 extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private BNO055IMU imu = null;
    private AnalogInput pixy_x;
    private AnalogInput pixy_object;

    @Override
    public void runOpMode()
    {
        frontLeftMotor  = hardwareMap.dcMotor.get("front_left_drive");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        backLeftMotor  = hardwareMap.dcMotor.get("back_left_drive");
        backRightMotor = hardwareMap.dcMotor.get("back_right_drive");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters1);
        pixy_x = hardwareMap.analogInput.get("pixy_x");
        pixy_object = hardwareMap.analogInput.get("pixy_object");
        AutonHelpers.init(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, imu, telemetry);

        double results[] = {3, 3, 3, 3, 3};
        int loopCount = 0;
        int colCount = 0;
        double average = 3;
        waitForStart();
        runtime.reset();

        AutonHelpers.moveLeft(0.27, 45, 0);
        while(opModeIsActive())
        {
            AutonHelpers.moveLeft(0.25, 45, 0);
            if (pixy_object.getVoltage() > 1)
            {
                results[loopCount % 5] = pixy_x.getVoltage();
                loopCount++;

                double newAverage = (results[0] + results[1] + results[2] + results[3] + results[4]) / 5;

                if (newAverage <= 1.4 && average > 1.4)
                {
                    colCount++;
                    sleep(500);
                }
                average = newAverage;
            }

            telemetry.addData("ColCount:", colCount);
            telemetry.addData("Average:", average);
            telemetry.addData("IsObject:", pixy_object.getVoltage());
            telemetry.addData("Coordinate:", pixy_x.getVoltage());
            telemetry.update();
            if (colCount == 2)
            {
                break;
            }
        }
    }
}