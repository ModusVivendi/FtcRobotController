package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadCalc {
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private OpMode opMode;
    private boolean isInitialized = false;

    // Constructor for both gamepads
    public GamepadCalc(Gamepad gamepad1, Gamepad gamepad2, OpMode opMode) {
        try {
            if (opMode == null) {
                throw new IllegalArgumentException("OpMode cannot be null");
            }
            this.gamepad1 = gamepad1;
            this.gamepad2 = gamepad2;
            this.opMode = opMode;
            this.isInitialized = true;
        } catch (Exception e) {
            this.isInitialized = false;
            if (opMode != null && opMode.telemetry != null) {
                opMode.telemetry.addData("GamepadCalc Error", "Initialization failed: " + e.getMessage());
                opMode.telemetry.update();
            }
        }
    }

    public void calculate() {
        try {
            if (!isInitialized) {
                throw new IllegalStateException("GamepadCalc not properly initialized");
            }
            if (opMode == null) {
                throw new IllegalStateException("OpMode is null");
            }
            gamepad1 = opMode.gamepad1;
            gamepad2 = opMode.gamepad2;
        } catch (Exception e) {
            if (opMode != null && opMode.telemetry != null) {
                opMode.telemetry.addData("GamepadCalc Error", "Calculate failed: " + e.getMessage());
                opMode.telemetry.update();
            }
        }
    }

//    Gamepad gamepad1;

    public Gamepad getGamepad1() {
        try {
            if (!isInitialized) {
                throw new IllegalStateException("GamepadCalc not properly initialized");
            }
            if (gamepad1 == null) {
                throw new IllegalStateException("Gamepad1 is null");
            }
            return gamepad1;
        } catch (Exception e) {
            if (opMode != null && opMode.telemetry != null) {
                opMode.telemetry.addData("GamepadCalc Error", "getGamepad1 failed: " + e.getMessage());
                opMode.telemetry.update();
            }
            return null;
        }
    }

    public Gamepad getGamepad2() {
        try {
            if (!isInitialized) {
                throw new IllegalStateException("GamepadCalc not properly initialized");
            }
            if (gamepad2 == null) {
                throw new IllegalStateException("Gamepad2 is null");
            }
            return gamepad2;
        } catch (Exception e) {
            if (opMode != null && opMode.telemetry != null) {
                opMode.telemetry.addData("GamepadCalc Error", "getGamepad2 failed: " + e.getMessage());
                opMode.telemetry.update();
            }
            return null;
        }
    }

    // Utility method to check if gamepads are working
    public boolean isGamepadSystemHealthy() {
        return isInitialized && gamepad1 != null && gamepad2 != null && opMode != null;
    }

    // Get detailed status
    public String getStatus() {
        StringBuilder status = new StringBuilder();
        status.append("Initialization: ").append(isInitialized ? "OK" : "Failed");
        status.append(", Gamepad1: ").append(gamepad1 != null ? "Connected" : "Disconnected");
        status.append(", Gamepad2: ").append(gamepad2 != null ? "Connected" : "Disconnected");
        return status.toString();
    }
}
