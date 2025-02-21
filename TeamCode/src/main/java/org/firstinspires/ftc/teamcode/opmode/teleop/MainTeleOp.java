package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.slides.VerticalSlides;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;

@TeleOp(name = "Main TeleOp")
@Disabled
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;
    private VerticalSlides slides;
    private RobotConfig robotConfig;

    private double targetHeading = 0.0;

    @Override
    public void init() {
        robotConfig = new RobotConfig(hardwareMap);
        drive = new MecanumDrive(robotConfig);
        slides = new VerticalSlides(robotConfig);
    }

    @Override
    public void loop() {
        // Get IMU heading (assuming you have IMU implementation)
        double currentHeading = 0.0; // Replace with actual IMU heading

        // Drive control (field-centric)
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;

        // Update target heading with right stick
        if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            targetHeading += gamepad1.right_stick_x * 0.05;
        }

        drive.driveFieldCentric(forward, strafe, targetHeading, currentHeading);

        // Slide control with different heights
        if (gamepad2.dpad_up) {
            slides.setHeight(FieldConfig.JUNCTION_HEIGHT_HIGH_MM, 1000);  // 1000mm/s velocity
        } else if (gamepad2.dpad_right) {
            slides.setHeight(FieldConfig.JUNCTION_HEIGHT_MEDIUM_MM, 800);
        } else if (gamepad2.dpad_down) {
            slides.setHeight(FieldConfig.JUNCTION_HEIGHT_LOW_MM, 600);
        }

        // Update slide control
        slides.update();

        // Telemetry
        telemetry.addData("Heading", "Target: %.1f, Current: %.1f",
                Math.toDegrees(targetHeading), Math.toDegrees(currentHeading));
        telemetry.addData("Slides", "Height: %.1f, At Target: %b",
                slides.getCurrentHeight(), slides.isAtTarget(5.0));
        telemetry.update();
    }
}