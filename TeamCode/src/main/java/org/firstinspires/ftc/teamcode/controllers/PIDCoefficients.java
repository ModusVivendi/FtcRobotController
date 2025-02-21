package org.firstinspires.ftc.teamcode.controllers;

/**
 * Data structure for PID controller coefficients
 */
public class PIDCoefficients {
    public double kP;
    public double kI;
    public double kD;

    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDCoefficients copy() {
        return new PIDCoefficients(kP, kI, kD);
    }

    @Override
    public String toString() {
        return String.format("PIDCoefficients (kP=%.3f, kI=%.3f, kD=%.3f)", kP, kI, kD);
    }
}