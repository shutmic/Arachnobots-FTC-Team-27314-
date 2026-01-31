package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.TurretShooter;
import org.firstinspires.ftc.teamcode.Subsystems.TurretSubsystem2;
import org.firstinspires.ftc.teamcode.Subsystems.TransferAndIntake;


@Autonomous(name="Full Forward + Turret Auto")
public class ArachnobotsAuton extends OpMode {

    private MecanumDrive drive;
    private TurretShooter turretShooter;
    private TurretSubsystem2 turretAim;
    private TransferAndIntake intake;

    private double startTime;
    private boolean turretFinished = false;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        turretShooter = new TurretShooter(hardwareMap);
        turretAim = new TurretSubsystem2(hardwareMap); // FIXED
        intake = new TransferAndIntake(hardwareMap);

        telemetry.addLine("Auton initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        startTime = getRuntime();
    }

    @Override
    public void loop() {
        double elapsed = getRuntime() - startTime;

        // ====== 1) Drive forward for 10 seconds ======
        if (elapsed < 10.0) {
            drive.setPowerAll(0.5, 0.5, 0.5, 0.5);
        } else {
            drive.stop();
        }

        // ====== 2) Rotate turret left 90 degrees ======
        if (!turretFinished) {
            double currentYaw = turretAim.imu.getRobotYawPitchRollAngles()
                    .getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            double targetYaw = -90; // rotate left 90 degrees
            double power = 0.3;

            if (currentYaw > targetYaw) {
                turretAim.setPower(-power);
            } else {
                turretAim.setPower(0);
                turretFinished = true;
            }
        }

        // ====== 3) Limelight + Shooter ======
        LLResult result = turretAim.limelight.getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
            turretAim.update(); // FIXED: call update(), not loop()

            // Get distance and calculate velocity
            double x = result.getFiducialResults().get(0).getRobotPoseTargetSpace().getPosition().x;
            double z = result.getFiducialResults().get(0).getRobotPoseTargetSpace().getPosition().z;
            double distance = Math.hypot(z, x);

            double targetVelocity = turretShooter.lookupVelocity(distance);
            targetVelocity = Math.min(targetVelocity, TurretShooter.MAX_TICKS_PER_SEC);

            // Ramp shooter smoothly
            if (turretShooter.currentTargetVelocity < targetVelocity) {
                turretShooter.currentTargetVelocity = Math.min(turretShooter.currentTargetVelocity + 300, targetVelocity);
            } else {
                turretShooter.currentTargetVelocity = Math.max(turretShooter.currentTargetVelocity - 300, targetVelocity);
            }

            turretShooter.shooterMotor.setVelocity(turretShooter.currentTargetVelocity);

        } else {
            // No target, ramp down
            turretShooter.currentTargetVelocity = Math.max(turretShooter.currentTargetVelocity - 300, 0);
            turretShooter.shooterMotor.setVelocity(turretShooter.currentTargetVelocity);
        }

        // ====== 4) Telemetry ======
        telemetry.addData("Elapsed Time", elapsed);
        telemetry.addData("Turret Finished", turretFinished);
        telemetry.addData("Turret Yaw", turretAim.imu.getRobotYawPitchRollAngles()
                .getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
        telemetry.addData("Shooter Velocity", turretShooter.currentTargetVelocity);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        turretShooter.stop();
        turretAim.stop();
        intake.stop();
    }
}
