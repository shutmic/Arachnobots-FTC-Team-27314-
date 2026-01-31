package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Turret Auto Aim (Limelight 3A)")
public class TurretSubsystem extends OpMode {

    // ---------------- HARDWARE ----------------
    private DcMotor turret;
    private Limelight3A limelight;
    private IMU imu;

    // ---------------- TUNING ----------------
    private static final double TURRET_KP = 0.02;      // P-gain
    private static final double TURRET_MAX_POWER = 0.4;
    private static final double TURRET_DEADBAND = 0.6; // degrees

    @Override
    public void init() {

        // Turret motor
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // turret.setDirection(DcMotor.Direction.REVERSE); // uncomment if reversed

        // Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // AprilTag pipeline

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("Turret Auto Aim Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update robot yaw for Limelight pose math
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        LLResult result = limelight.getLatestResult();
        double turretPower = 0.0;

        if (result != null && result.isValid()) {
            double tx = result.getTx(); // horizontal angle offset in degrees

            if (Math.abs(tx) > TURRET_DEADBAND) {
                turretPower = -tx * TURRET_KP;
            }

            // Clamp power
            turretPower = Math.max(-TURRET_MAX_POWER,
                    Math.min(TURRET_MAX_POWER, turretPower));

            telemetry.addData("Target", "Visible");
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("Turret Power", turretPower);
        } else {
            telemetry.addData("Target", "None");
            turretPower = 0.0;
        }

        turret.setPower(turretPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }
}
