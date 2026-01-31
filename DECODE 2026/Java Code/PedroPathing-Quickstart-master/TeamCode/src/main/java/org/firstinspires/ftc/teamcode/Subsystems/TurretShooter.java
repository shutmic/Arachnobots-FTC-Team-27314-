package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class TurretShooter {
    public DcMotorEx shooterMotor;
    public Limelight3A limelight;

    public static final double MAX_RPM = 6000;
    public static final double TICKS_PER_REV = 28;
    public static final double MAX_TICKS_PER_SEC = (MAX_RPM / 60.0) * TICKS_PER_REV;

    private static final double kP = 0.0020, kI = 0.0, kD = 0.0001;
    private static final double kF = 32767.0 / MAX_TICKS_PER_SEC;
    private static final double RAMP_RATE = 300;

    public double currentTargetVelocity = 0;

    private double[][] lookupTable = { {0.5,1500}, {1.0,4000}, {2.0,6000} };

    public TurretShooter(HardwareMap hw) {
        shooterMotor = hw.get(DcMotorEx.class, "shooterMotor");
        limelight = hw.get(Limelight3A.class, "limelight");

        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        limelight.start();
    }

    public void update(boolean triggerPressed) {
        double distance = 0;
        double targetTicks = 0;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
            Pose3D tagPose = result.getFiducialResults().get(0).getRobotPoseTargetSpace();
            double x = tagPose.getPosition().x;
            double z = tagPose.getPosition().z;
            distance = Math.hypot(z, x);
            targetTicks = lookupVelocity(distance);
            targetTicks = Math.min(targetTicks, MAX_TICKS_PER_SEC);
        }

        if (triggerPressed) {
            if (currentTargetVelocity < targetTicks) currentTargetVelocity = Math.min(currentTargetVelocity + RAMP_RATE, targetTicks);
            else currentTargetVelocity = Math.max(currentTargetVelocity - RAMP_RATE, targetTicks);
        } else {
            currentTargetVelocity = Math.max(currentTargetVelocity - RAMP_RATE, 0);
        }

        shooterMotor.setVelocity(currentTargetVelocity);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    public double lookupVelocity(double distance) {
        if (distance <= lookupTable[0][0]) return lookupTable[0][1];
        if (distance >= lookupTable[lookupTable.length-1][0]) return lookupTable[lookupTable.length-1][1];
        for (int i=0;i<lookupTable.length-1;i++) {
            double d1 = lookupTable[i][0], d2 = lookupTable[i+1][0];
            if (distance >= d1 && distance <= d2) {
                double v1 = lookupTable[i][1], v2 = lookupTable[i+1][1];
                double ratio = (distance - d1)/(d2-d1);
                return v1 + ratio*(v2-v1);
            }
        }
        return lookupTable[lookupTable.length-1][1];
    }
}
