package org.firstinspires.ftc.teamcode.RobotTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PIDF Turret 2")
public class Test extends OpMode {


    private DcMotorEx shooter;

    // PID control variables
    private double shooterTargetRPM = 0;
    private double shooterErrorSum = 0;
    private final double shooterKp = 0.0007; // small P term
    private final double shooterKi = 0.00008; // I term dominates
    private final double shooterMaxPower = 1.0;

    // Shooter constants
    private final double SHOOTER_ANGLE = Math.toRadians(45); // fixed launch angle
    private final double SHOOTER_HEIGHT = 0.4; // meters
    private final double GRAVITY = 9.81; // m/s^2
    private final double GOAL_HEIGHT = 1.1; // meters

    // Yellow Jacket max RPM
    private final double MOTOR_MAX_RPM = 6000;

    // Linear velocity per RPM (tune empirically)
    private final double shooterRPMtoVelocity = 0.00075; // m/s per RPM (example)

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double distanceToGoal = getDistanceFromLimelight();

        if (distanceToGoal > 0) {
            // 1 Calculate required projectile velocity
            double v0 = calculateV0(distanceToGoal);

            // 2️ Convert to flywheel RPM
            shooterTargetRPM = velocityToRPM(v0);

            // Clamp RPM to Yellow Jacket max
            shooterTargetRPM = Math.min(shooterTargetRPM, MOTOR_MAX_RPM);

            // 3️ Spin up flywheel using PID
            double power = flywheelPID(shooter.getVelocity());
            shooter.setPower(power);
        } else {
            shooter.setPower(0);
        }

        telemetry.addData("Distance (m)", distanceToGoal);
        telemetry.addData("Target RPM", shooterTargetRPM);
        telemetry.addData("Actual RPM", shooter.getVelocity());
        telemetry.update();
    }

    // Limelight placeholder: replace with real network table read
    private double getDistanceFromLimelight() {
        double limelightYAngle = 0; // get from Limelight: ty
        double cameraAngle = 25;    // Limelight mounting angle
        double totalAngle = Math.toRadians(limelightYAngle + cameraAngle);

        if (totalAngle <= 0) return -1;

        return (GOAL_HEIGHT - SHOOTER_HEIGHT) / Math.tan(totalAngle);
    }

    private double calculateV0(double distance) {
        double yDiff = GOAL_HEIGHT - SHOOTER_HEIGHT;
        double cosTheta = Math.cos(SHOOTER_ANGLE);
        double sinTheta = Math.sin(SHOOTER_ANGLE);

        double numerator = GRAVITY * distance * distance;
        double denominator = 2 * cosTheta * cosTheta * (distance * Math.tan(SHOOTER_ANGLE) - yDiff);

        return Math.sqrt(numerator / denominator);
    }

    private double velocityToRPM(double v0) {
        return v0 / shooterRPMtoVelocity;
    }

    private double flywheelPID(double currentRPM) {
        double error = shooterTargetRPM - currentRPM;
        shooterErrorSum += error;
        double power = shooterKp * error + shooterKi * shooterErrorSum;

        // Clamp power to 0–1
        return Math.max(Math.min(power, shooterMaxPower), 0);
    }
}
