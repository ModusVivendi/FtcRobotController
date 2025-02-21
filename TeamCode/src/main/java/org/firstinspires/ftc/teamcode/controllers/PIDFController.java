package org.firstinspires.ftc.teamcode.controllers;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Extended PID controller with feedforward components
 */
public final class PIDFController extends PIDController {
    public interface FeedforwardFunction {
        double compute(double position, @Nullable Double velocity);
    }

    // Feedforward components
    public final double kV;
    public final double kA;
    public final double kStatic;
    public final FeedforwardFunction kF;

    // Additional targets for feedforward
    public double targetVelocity;
    public double targetAcceleration;

    // Integral bounds
    private double integralLowerBound = Double.NEGATIVE_INFINITY;
    private double integralUpperBound = Double.POSITIVE_INFINITY;


    /**
     * Creates a PIDF controller with full feedforward capabilities
     */
    public PIDFController(
            @NonNull PIDCoefficients pid,
            double kV,
            double kA,
            double kStatic,
            @NonNull FeedforwardFunction kF
    ) {
        super(pid);
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
    }

    /**
     * Creates a PIDF controller with basic feedforward
     */
    public PIDFController(
            @NonNull PIDCoefficients pid,
            double kV,
            double kA,
            double kStatic
    ) {
        this(pid, kV, kA, kStatic, (x, v) -> 0);
    }

    /**
     * Creates a PIDF controller with custom feedforward
     */
    public PIDFController(
            @NonNull PIDCoefficients pid,
            @NonNull FeedforwardFunction kF
    ) {
        this(pid, 0, 0, 0, kF);
    }

    /**
     * Sets bounds on the integral term to prevent windup
     * @param lowerBound Lower bound for integral sum
     * @param upperBound Upper bound for integral sum
     */
    public void setIntegralBounds(double lowerBound, double upperBound) {
        this.integralLowerBound = lowerBound;
        this.integralUpperBound = upperBound;
        // Clamp current errorSum if it's outside the new bounds
        errorSum = Math.min(Math.max(errorSum, integralLowerBound), integralUpperBound);
    }

    @Override
    public double update(long timestamp, double measuredPosition) {
        return update(timestamp, measuredPosition, null);
    }

    /**
     * Updates the controller with measured position and optional velocity
     */
    public double update(long timestamp, double measuredPosition, @Nullable Double measuredVelocity) {
        final double error = getPositionError(measuredPosition);

        if (lastUpdateTime == 0) {
            lastError = error;
            lastUpdateTime = timestamp;
            return 0;
        }

        final double dt = (timestamp - lastUpdateTime) / 1e9;
        errorSum += 0.5 * (error + lastError) * dt;
        final double errorDeriv = (error - lastError) / dt;

        lastError = error;
        lastUpdateTime = timestamp;

        // Use measured velocity if available, otherwise use derivative
        double velError = (measuredVelocity == null) ? errorDeriv : targetVelocity - measuredVelocity;

        // Calculate PID terms
        double baseOutput = coefficients.kP * error +
                coefficients.kI * errorSum +
                coefficients.kD * velError;

        // Add feedforward terms
        baseOutput += kV * targetVelocity +
                kA * targetAcceleration +
                kF.compute(measuredPosition, measuredVelocity);

        // Apply static friction compensation
        double output = 0;
        if (Math.abs(baseOutput) > 1e-6) {
            output = baseOutput + Math.copySign(kStatic, baseOutput);
        }

        // Apply output bounds if set
        if (outputBounded) {
            output = Math.max(minOutput, Math.min(output, maxOutput));
        }

        return output;
    }

    @Override
    public void reset() {
        super.reset();
        targetVelocity = 0;
        targetAcceleration = 0;
        errorSum = 0; // Reset integral term
    }

    /**
     * Gets the current integral term value
     */
    public double getIntegralTerm() {
        return errorSum;
    }

}