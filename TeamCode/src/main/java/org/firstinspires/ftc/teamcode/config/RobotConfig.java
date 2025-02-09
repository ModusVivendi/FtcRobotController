package org.firstinspires.ftc.teamcode.config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Functions.ColorSensorV3;

public class RobotConfig {
    public static class HardwareConfig {
        // Motors
        public static final boolean ENABLE_FL = true;
        public static final boolean ENABLE_FR = true;
        public static final boolean ENABLE_BL = true;
        public static final boolean ENABLE_BR = true;

        // Vertical Slider Motors (GoBilda Yellow Jacket)
        public static final boolean ENABLE_VERT_SLIDE_LEFT = true;
        public static final boolean ENABLE_VERT_SLIDE_RIGHT = true;

        // Horizontal Slider Servos
        public static final boolean ENABLE_HORIZ_SLIDE_LEFT = true;
        public static final boolean ENABLE_HORIZ_SLIDE_RIGHT = true;

        // Vertical Claw Servos
        public static final boolean ENABLE_VERT_CLAW_ROTATE_LEFT = true;
        public static final boolean ENABLE_VERT_CLAW_ROTATE_RIGHT = true;
        public static final boolean ENABLE_VERT_CLAW_GRIPPER = true;

        // Horizontal Claw Servos
        public static final boolean ENABLE_HORIZ_CLAW_ROTATE_LEFT = true;
        public static final boolean ENABLE_HORIZ_CLAW_ROTATE_RIGHT = true;
        public static final boolean ENABLE_HORIZ_CLAW_GRIPPER = true;

        // Digital sensors
        public static final boolean ENABLE_HORIZ_CLAW_COLOR = true;

        // old config starts here
        public static final boolean ENABLE_SLIDE_LEFT = false;
        public static final boolean ENABLE_SLIDE_RIGHT = false;
        public static final boolean ENABLE_ROTATE_LEFT = false;
        public static final boolean ENABLE_ROTATE_RIGHT = false;

        // Servos
        public static final boolean ENABLE_LEFT_AXLE = false;
        public static final boolean ENABLE_RIGHT_AXLE = false;
        public static final boolean ENABLE_LEFT_GECKO = false;
        public static final boolean ENABLE_RIGHT_GECKO = false;
        public static final boolean ENABLE_LEFT_HORIZ_SLIDE = false;
        public static final boolean ENABLE_RIGHT_HORIZ_SLIDE = false;
        public static final boolean ENABLE_HORIZ_CLAW = false;
        public static final boolean ENABLE_VERT_CLAW = false;
        // Old config ends here
    }

    public HardwareMap hardwareMap;

    public RobotConfig(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        this.hardwareMap = hardwareMap;
    }

    public DcMotor getMotorIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.dcMotor.get(name) : null;
    }

    public DcMotorEx getMotorExIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.get(DcMotorEx.class, name) : null;
    }

    public Servo getServoIfEnabled(String name, boolean isEnabled) {
        return isEnabled ? hardwareMap.servo.get(name) : null;
    }

    // Add getter for hardwareMap if needed
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public ColorSensorV3 getColorSensorIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        ColorSensor rawSensor = hardwareMap.get(ColorSensor.class, name);
        return rawSensor != null ? new ColorSensorV3(rawSensor) : null;
    }
}