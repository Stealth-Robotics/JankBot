package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Michael Vierra, FTC 8461 on 9/13/2017.
 */

/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */

@TeleOp(name = "PixyI2C", group = "SensorTest")
public class PixyI2C extends LinearOpMode {
    private I2cDeviceSynch pixy;

    public Block[] blockList = new Block[135];
    //our Pixy device
    @Override
    public void runOpMode() throws InterruptedException {
        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");

        //setting Pixy's I2C Address
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the SDK Documentation to learn more
        //I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26, I2cDeviceSynch.ReadMode.REPEAT);
        //pixy.setReadWindow(readWindow);

        //required to "turn on" the device
        pixy.engage();

        int block = 0;
        int checksum;
        int signature;
        int x;
        int y;
        int width;
        int height;

        waitForStart();

        while(opModeIsActive()) {
            byte read8 = pixy.read8(0);
            if ((read8 & 0xff) != 0x55)
            {
                telemetry.addData("Block", "Not Found");
                continue;
            }
            byte read8_2 = pixy.read8(0);
            if ((read8_2 & 0xff) == 0xaa)
            {
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                if ((read8 & 0xff) == 0x55 && (read8_2 & 0xff) == 0xaa)
                {
                    telemetry.addData("SyncCount", "2");
                    block = 0;
                    read8 = pixy.read8(0);
                    read8_2 = pixy.read8(0);
                    checksum = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                }
                else
                {
                    telemetry.addData("SyncCount", "1");
                    checksum = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                    block++;
                }
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                signature = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                x = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                y = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                width = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                read8 = pixy.read8(0);
                read8_2 = pixy.read8(0);
                height = (read8 & 0xff) + ((read8_2 & 0xff) << 8);
                telemetry.addData("Checksum", checksum);
                telemetry.addData("Block", block);
                telemetry.addData("Signature", signature);
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.addData("Width", width);
                telemetry.addData("Height", height);
                if (signature + x + y + width + height == checksum)
                {
                    blockList[block] = new Block(signature, x, y, width, height);
                    telemetry.addData("Added", true);
                }
                else
                {
                    telemetry.addData("Added", false);
                }
            }
            telemetry.update();
        }
    }
}
