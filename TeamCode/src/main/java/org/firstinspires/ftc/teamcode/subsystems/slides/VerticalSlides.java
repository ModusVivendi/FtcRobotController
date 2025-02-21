package org.firstinspires.ftc.teamcode.subsystems.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.HardwareConfig.VerticalSlideConfig;
import org.firstinspires.ftc.teamcode.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;

public class VerticalSlides {
    private final DcMotorEx leftSlide, rightSlide;
    private final PIDFController slideController;

    // PIDF coefficients for position control
    private static final PIDCoefficients SLIDE_PID = new PIDCoefficients(0.015, 0.001, 0.001);
    private static final double kV = 0.01;  // Velocity feedforward
    private static final double kA = 0.0;   // Acceleration feedforward
    private static final double kStatic = 0.05;  // Static friction compensation

    private double targetHeight = 0.0;
    private static final double TICKS_PER_MM = VerticalSlideConfig.TICKS_PER_REV /
            (Math.PI * VerticalSlideConfig.SPOOL_DIAMETER_MM);

    public VerticalSlides(RobotConfig config) {
        leftSlide = config.getMotorExIfEnabled(
                VerticalSlideConfig.LEFT_NAME,
                VerticalSlideConfig.ENABLE_LEFT
        );

        rightSlide = config.getMotorExIfEnabled(
                VerticalSlideConfig.RIGHT_NAME,
                VerticalSlideConfig.ENABLE_RIGHT
        );

        // Initialize PIDF controller
        slideController = new PIDFController(SLIDE_PID, kV, kA, kStatic);
        slideController.setOutputBounds(-1, 1);

        // Reset encoders
        if (leftSlide != null) {
            leftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightSlide != null) {
            rightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setHeight(double heightMM, double velocity) {
        targetHeight = Math.min(Math.max(heightMM,
                        VerticalSlideConfig.MIN_HEIGHT_MM),
                VerticalSlideConfig.MAX_HEIGHT_MM
        );

        slideController.setTargetPosition(targetHeight);
        slideController.targetVelocity = velocity;
    }

    public void update() {
        if (leftSlide == null || rightSlide == null) return;

        // Get current position (average of both slides)
        double currentPosition = (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition())
                / (2.0 * TICKS_PER_MM);

        // Get current velocity
        double currentVelocity = (leftSlide.getVelocity() + rightSlide.getVelocity())
                / (2.0 * TICKS_PER_MM);

        // Calculate motor power using PIDF
        double power = slideController.update(
                System.nanoTime(),
                currentPosition,
                currentVelocity
        );

        // Apply power to both motors
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public boolean isAtTarget(double tolerance) {
        return slideController.atTargetPosition(tolerance);
    }

    public double getCurrentHeight() {
        if (leftSlide == null || rightSlide == null) return 0;
        return (leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition())
                / (2.0 * TICKS_PER_MM);
    }
}