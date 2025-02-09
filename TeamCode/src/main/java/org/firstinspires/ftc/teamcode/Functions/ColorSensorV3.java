package org.firstinspires.ftc.teamcode.Functions;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ColorSensorV3 {
    private ColorSensor colorSensor;
    private static final int[] TARGET_COLOR = {200, 200, 0}; // Target RGB for distance calc
    private static final int SAMPLES_FOR_CALIBRATION = 5;  // Number of readings to average
    // Sensitivity configuration
    private double redSensitivity = 1.0;
    private double blueSensitivity = 1.0;
    private double yellowSensitivity = 1.0;
    private double whiteSensitivity = 1.0;
    private double distanceMultiplier = 0.11;   // Distance calibration factor

    // LED state tracking
    private boolean ledState = false;


    // Constructor with HardwareMap
    public ColorSensorV3(HardwareMap hardwareMap, String name) {
        try {
            colorSensor = hardwareMap.get(ColorSensor.class, name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize Color Sensor: " + e.getMessage());
        }
    }

    // Constructor with direct ColorSensor
    public ColorSensorV3(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    // Sensitivity adjustment
    public void setRedSensitivity(double sensitivity) {
        this.redSensitivity = Range.clip(sensitivity, 0.1, 2.0);
    }

    public void setBlueSensitivity(double sensitivity) {
        this.blueSensitivity = Range.clip(sensitivity, 0.1, 2.0);
    }

    public void setYellowSensitivity(double sensitivity) {
        this.yellowSensitivity = Range.clip(sensitivity, 0.1, 2.0);
    }

    public void setWhiteSensitivity(double sensitivity) {
        this.whiteSensitivity = Range.clip(sensitivity, 0.1, 2.0);
    }

    private boolean isValidHSV(float[] hsv) {
        // Check if HSV values are within expected ranges
        return hsv != null &&
                hsv.length == 3 &&
                !Float.isNaN(hsv[0]) &&
                !Float.isNaN(hsv[1]) &&
                !Float.isNaN(hsv[2]) &&
                hsv[0] >= 0 && hsv[0] <= 360 &&
                hsv[1] >= 0 && hsv[1] <= 1 &&
                hsv[2] >= 0 && hsv[2] <= 1;
    }

    // Distance calculation using color difference
    public double getDistance() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        int colorDifference = Math.abs(r - TARGET_COLOR[0]) +
                Math.abs(g - TARGET_COLOR[1]) +
                Math.abs(b - TARGET_COLOR[2]);
        return colorDifference * 0.5 * distanceMultiplier;
    }

    // Averaged distance reading
    public double getAveragedDistance(int samples) {
        double sum = 0;
        for(int i = 0; i < samples; i++) {
            sum += getDistance();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                break;
            }
        }
        return sum / samples;
    }

    // Get HSV values for better color detection
    public float[] getHSV() {
        float[] hsv = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsv);
        return hsv;
    }

    // Raw color values
    public int getRed() { return colorSensor.red(); }
    public int getGreen() { return colorSensor.green(); }
    public int getBlue() { return colorSensor.blue(); }
    public int getAlpha() { return colorSensor.alpha(); }

    // Normalized RGB values
    public double[] getNormalizedRGB() {
        double sum = getRed() + getGreen() + getBlue();
        if (sum == 0) return new double[]{0, 0, 0};

        return new double[]{
                getRed() / sum,
                getGreen() / sum,
                getBlue() / sum
        };
    }

    // Color detection with HSV (more reliable than RGB)
    public boolean isRed() {
        float[] hsv = getHSV();
        if (!isValidHSV(hsv)) {
            return false; // Invalid HSV values
        }
        // Adjusted with sensitivity multiplier
        return (hsv[0] >= 340 || hsv[0] <= 20) &&
                hsv[1] > 0.5 * redSensitivity &&
                hsv[2] > 0.2 * redSensitivity;
    }

    public boolean isBlue() {
        float[] hsv = getHSV();
        if (!isValidHSV(hsv)) {
            return false; // Invalid HSV values
        }
        // Adjusted with sensitivity multiplier
        return (hsv[0] >= 200 && hsv[0] <= 250) &&
                hsv[1] > 0.5 * blueSensitivity &&
                hsv[2] > 0.2 * blueSensitivity;
    }

    public boolean isYellow() {
        float[] hsv = getHSV();
        if (!isValidHSV(hsv)) {
            return false; // Invalid HSV values
        }
        // Adjusted with sensitivity multiplier
        return (hsv[0] >= 50 && hsv[0] <= 60) &&
                hsv[1] > 0.5 * yellowSensitivity &&
                hsv[2] > 0.2 * yellowSensitivity;
    }

    public boolean isWhite() {
        float[] hsv = getHSV();
        if (!isValidHSV(hsv)) {
            return false; // Invalid HSV values
        }
        // Adjusted with sensitivity multiplier
        return hsv[1] < 0.2 * whiteSensitivity &&
                hsv[2] > 0.8 * whiteSensitivity;
    }

    // Color confidence level
    public double getColorConfidence(String color) {
        float[] hsv = getHSV();
        if (!isValidHSV(hsv)) {
            return 0; // Invalid HSV values
        }
        switch (color.toUpperCase()) {
            case "RED":
                if (hsv[0] >= 340 || hsv[0] <= 20)
                    return hsv[1] * hsv[2] * redSensitivity;
                return 0;
            case "BLUE":
                if (hsv[0] >= 200 && hsv[0] <= 250)
                    return hsv[1] * hsv[2] * blueSensitivity;
                return 0;
            case "WHITE":
                if (hsv[1] < 0.2)
                    return hsv[2] * whiteSensitivity;
                return 0;
            case "YELLOW":
                if (hsv[0] >= 50 && hsv[0] <= 60) {
                    return hsv[1] * hsv[2] * yellowSensitivity;
                }
                return 0;
            default:
                return 0;
        }
    }

    // Get color info string for telemetry
    public String getColorInfo() {
        float[] hsv = getHSV();
        StringBuilder info = new StringBuilder();

        // RGB values
        info.append("RGB:[")
                .append(getRed()).append(',')
                .append(getGreen()).append(',')
                .append(getBlue()).append("] ");

        // HSV values
        info.append("HSV:[")
                .append((int)hsv[0]).append(',')
                .append(String.format("%.2f", hsv[1])).append(',')
                .append(String.format("%.2f", hsv[2])).append("] ");

        // Distance
        info.append("Dist:")
                .append(String.format("%.2f", getDistance()));

        return info.toString();
    }

    // Status check methods
    public boolean isOperational() {
        return colorSensor != null &&
                (getRed() + getGreen() + getBlue()) > 0;
    }

    public String getStatus() {
        StringBuilder status = new StringBuilder();
        status.append("Sensor: ").append(isOperational() ? "OK" : "ERROR")
                .append(" LED: ").append(ledState ? "ON" : "OFF")
                .append(" Raw: ").append(getColorInfo());
        return status.toString();
    }

    // LED control methods
    public void enableLED(boolean enable) {
        colorSensor.enableLed(enable);
        ledState = enable;
    }

    public boolean getLED() {
        return ledState;
    }

    // Calibration methods
    public void calibrateDistanceMultiplier(double knownDistance) {
        int totalColorDifference = 0;

        // Take multiple samples to get a more stable reading
        for (int i = 0; i < SAMPLES_FOR_CALIBRATION; i++) {
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            totalColorDifference += Math.abs(r - TARGET_COLOR[0]) +
                    Math.abs(g - TARGET_COLOR[1]) +
                    Math.abs(b - TARGET_COLOR[2]);

            // Short delay between readings
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                break;
            }
        }

        // Calculate average color difference
        double avgColorDifference = totalColorDifference / (double)SAMPLES_FOR_CALIBRATION;

        // Calculate new multiplier based on known distance
        this.distanceMultiplier = knownDistance / (avgColorDifference * 0.5);
    }
}
