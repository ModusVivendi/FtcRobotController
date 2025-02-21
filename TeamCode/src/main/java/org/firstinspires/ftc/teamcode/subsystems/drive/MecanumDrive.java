package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.HardwareConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controllers.PIDCoefficients;

public class MecanumDrive {
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final PIDController headingController;

    // PID coefficients for heading control
    private static final PIDCoefficients HEADING_PID = new PIDCoefficients(0.05, 0.0, 0.002);

    public MecanumDrive(RobotConfig config) {
        frontLeft = config.getDriveMotor(
                DrivetrainConfig.FL_NAME,
                DrivetrainConfig.ENABLE_FL,
                DrivetrainConfig.FL_REVERSE
        );

        frontRight = config.getDriveMotor(
                DrivetrainConfig.FR_NAME,
                DrivetrainConfig.ENABLE_FR,
                DrivetrainConfig.FR_REVERSE
        );

        backLeft = config.getDriveMotor(
                DrivetrainConfig.BL_NAME,
                DrivetrainConfig.ENABLE_BL,
                DrivetrainConfig.BL_REVERSE
        );

        backRight = config.getDriveMotor(
                DrivetrainConfig.BR_NAME,
                DrivetrainConfig.ENABLE_BR,
                DrivetrainConfig.BR_REVERSE
        );

        // Initialize heading PID controller
        headingController = new PIDController(HEADING_PID);
        headingController.setInputBounds(-Math.PI, Math.PI);  // Angle bounds
        headingController.setOutputBounds(-1, 1);  // Motor power bounds
    }

    public void driveFieldCentric(double drive, double strafe, double targetHeading, double currentHeading) {
        // Calculate heading correction using PID
        headingController.setTargetPosition(targetHeading);
        double headingCorrection = headingController.update(currentHeading);

        // Calculate wheel powers
        double sin = Math.sin(currentHeading);
        double cos = Math.cos(currentHeading);

        double rotX = strafe * cos - drive * sin;
        double rotY = strafe * sin + drive * cos;

        setMotorPowers(
                rotY + rotX + headingCorrection,  // Front Left
                rotY - rotX - headingCorrection,  // Front Right
                rotY - rotX + headingCorrection,  // Back Left
                rotY + rotX - headingCorrection   // Back Right
        );
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        double max = Math.max(1.0, Math.max(
                Math.abs(fl), Math.max(
                        Math.abs(fr), Math.max(
                                Math.abs(bl), Math.abs(br)
                        )
                )
        ));

        if (frontLeft != null) frontLeft.setPower(fl / max);
        if (frontRight != null) frontRight.setPower(fr / max);
        if (backLeft != null) backLeft.setPower(bl / max);
        if (backRight != null) backRight.setPower(br / max);
    }
}
