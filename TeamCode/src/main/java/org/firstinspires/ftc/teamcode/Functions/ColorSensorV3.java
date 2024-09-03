package org.firstinspires.ftc.teamcode.Functions;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorV3 {
    private ColorSensor colorSensor;

    public ColorSensorV3(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public double getDistance() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int[] colorThreshold = {200, 200, 0};
        int colorDifference = Math.abs(r - colorThreshold[0]) + Math.abs(g - colorThreshold[1]) + Math.abs(b - colorThreshold[2]);
        double distance = colorDifference * 0.5 * 0.11;

        return distance;
    }
}

