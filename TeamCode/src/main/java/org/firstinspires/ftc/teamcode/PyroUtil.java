package org.firstinspires.ftc.teamcode;

public class PyroUtil {
    public static double clamp(double number, double min, double max) {
        if (number < min) {
            return min;
        }
        else if (number > max) {
            return max;
        }
        else {
            return number;
        }
    }
}
