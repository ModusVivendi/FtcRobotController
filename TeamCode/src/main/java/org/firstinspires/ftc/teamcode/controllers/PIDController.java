package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;

/**
 * Basic PID Controller implementation
 */
public class PIDController {
    // Core PID components
    public final PIDCoefficients coefficients;
    public double errorSum;
    public double lastError;
    public long lastUpdateTime;

    // Bounds
    public boolean inputBounded;
    public double minInput, maxInput;
    public boolean outputBounded;
    public double minOutput, maxOutput;

    // Target
    public double targetPosition; // Target position (that is, the controller setpoint)

    /**
     * Feedforward parameters {@code kV}, {@code kA}, and {@code kStatic} correspond with a basic
     * kinematic model of DC motors. The general function {@code kF} computes a custom feedforward
     * term for other plants.
     *
     * @param coefficients traditional PID coefficients
     */
    public PIDController(@NonNull PIDCoefficients coefficients) {
        this.coefficients = coefficients;
        reset();
    }

    /**
     * Sets the target position (setpoint) for the controller
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    /**
     * Sets bound on the input of the controller. When computing the error, the min and max are
     * treated as the same value. (Imagine taking the segment of the real line between min and max
     * and attaching the endpoints.)
     *
     * @param min minimum input
     * @param max maximum input
     */
    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    /**
     * Sets bounds on the output of the controller.
     *
     * @param min minimum output
     * @param max maximum output
     */
    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    /**
     * Calculate position error considering input bounds if set
     */
    public double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2.0) {
                error -= Math.copySign(inputRange, error);
            }
        }
        return error;
    }

    /**
     * Update the controller with the current position
     */
    public double update(double measuredPosition) {
        return update(System.nanoTime(), measuredPosition);
    }

    /**
     * Update the controller with the current position and timestamp
     */
    public double update(long timestamp, double measuredPosition) {
        double error = getPositionError(measuredPosition);

        if (lastUpdateTime == 0) {
            lastError = error;
            lastUpdateTime = timestamp;
            return 0;
        }

        // Convert nanoseconds to seconds for time delta
        double dt = (timestamp - lastUpdateTime) / 1e9;

        // Calculate integral and derivative terms
        errorSum += 0.5 * (error + lastError) * dt;
        double errorRate = (error - lastError) / dt;

        // Store values for next iteration
        lastError = error;
        lastUpdateTime = timestamp;

        // Calculate output using PID formula
        double output = coefficients.kP * error +
                coefficients.kI * errorSum +
                coefficients.kD * errorRate;

        // Apply output bounds if set
        if (outputBounded) {
            output = Math.max(minOutput, Math.min(output, maxOutput));
        }

        return output;
    }

    /**
     * Check if the controller is at the target position
     */
    public boolean atTargetPosition(double tolerance) {
        return Math.abs(lastError) <= tolerance;
    }

    /**
     * Reset the controller's state
     */
    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastUpdateTime = 0;
    }

    /**
     * Get the current target position
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Get the last calculated error
     */
    public double getLastError() {
        return lastError;
    }
}

