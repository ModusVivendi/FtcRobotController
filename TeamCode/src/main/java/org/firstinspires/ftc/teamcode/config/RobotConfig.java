package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.Functions.ColorSensorV3;

public class RobotConfig {
    private final HardwareMap hardwareMap;

    public RobotConfig(HardwareMap hardwareMap) {
        if (hardwareMap == null) {
            throw new IllegalArgumentException("HardwareMap cannot be null");
        }
        this.hardwareMap = hardwareMap;
    }

    // Generic hardware getters with error handling
    public DcMotor getMotorIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize motor: " + name, e);
        }
    }

    public DcMotorEx getMotorExIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize motorEx: " + name, e);
        }
    }

    public Servo getServoIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            return hardwareMap.servo.get(name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize servo: " + name, e);
        }
    }

    public ColorSensorV3 getColorSensorIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            ColorSensor rawSensor = hardwareMap.get(ColorSensor.class, name);
            return rawSensor != null ? new ColorSensorV3(rawSensor) : null;
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize color sensor: " + name, e);
        }
    }

    public DistanceSensor getDistanceSensorIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            return hardwareMap.get(DistanceSensor.class, name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize distance sensor: " + name, e);
        }
    }

    public IMU getIMUIfEnabled(String name, boolean isEnabled) {
        if (!isEnabled) return null;
        try {
            return hardwareMap.get(IMU.class, name);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize IMU: " + name, e);
        }
    }

    // Convenience methods for specific subsystems
    public DcMotorEx getDriveMotor(String name, boolean isEnabled, boolean reverse) {
        DcMotorEx motor = getMotorExIfEnabled(name, isEnabled);
        if (motor != null) {
            motor.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return motor;
    }

    public DcMotorEx getSlideMotor(String name, boolean isEnabled, boolean reverse) {
        DcMotorEx motor = getMotorExIfEnabled(name, isEnabled);
        if (motor != null) {
            motor.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPositionPIDFCoefficients(HardwareConfig.VerticalSlideConfig.kP);
        }
        return motor;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
}