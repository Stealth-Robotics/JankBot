package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;


public class AutonHelpers {

    private static DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private static BNO055IMU imu;
    private static Telemetry telemetry;

    // todo: write your code here
    public static void init(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, BNO055IMU imu, Telemetry telemetry)
    {
        frontLeftMotor = frontLeft;
        frontRightMotor = frontRight;
        backLeftMotor = backLeft;
        backRightMotor = backRight;
        AutonHelpers.imu = imu;
        AutonHelpers.telemetry = telemetry;
    }

    private static void driveWithCorrection(double flPow, double blPow, double frPow, double brPow,
                                            double angleCorrectionCoeff, double angleTarget)
    {
        telemetry.addData("Roll: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
        telemetry.update();

        double frontLeftPower = flPow;
        double frontRightPower = frPow;
        double backLeftPower = blPow;
        double backRightPower = brPow;

        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - angleTarget;
        angle /= angleCorrectionCoeff;
        angle = ((angle > -1) ? ((angle < 1) ? angle : 1) : -1);
        frontLeftPower += angle;
        backLeftPower += angle;
        frontRightPower -= angle;
        backRightPower -= angle;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public static void moveLeft(double power, double correction, double angle)
    {
        driveWithCorrection(-power, power, power, -power, correction, angle);
    }

    public static void moveRight(double power, double correction, double angle)
    {
        driveWithCorrection(power, -power, -power, power, correction, angle);
    }

    public static void moveForward(double power, double correction, double angle)
    {
        driveWithCorrection(power, power, power, power, correction, angle);
    }

    public static void moveBack(double power, double correction, double angle)
    {
        driveWithCorrection(-power, -power, -power, -power, correction, angle);
    }

    public static void rotateTo(double power, double angle)
    {
        if (angle < imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)
        {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
        }
        else
        {
            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
        }
    }

    public static void stop()
    {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}