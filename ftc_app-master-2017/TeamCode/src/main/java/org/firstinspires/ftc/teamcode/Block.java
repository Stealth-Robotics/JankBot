package org.firstinspires.ftc.teamcode;


public class Block {
    public int signature;
    public int x;
    public int y;
    public int width;
    public int height;

    Block(int sig, int x_val, int y_val, int width_val, int height_val)
    {
        signature = sig;
        x = x_val;
        y = y_val;
        width = width_val;
        height = height_val;
    }
}