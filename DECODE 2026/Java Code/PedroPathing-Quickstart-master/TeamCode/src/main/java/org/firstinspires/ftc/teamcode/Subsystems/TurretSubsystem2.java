package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurretSubsystem2 {

    public DcMotor turret;
    public Limelight3A limelight;
    public IMU imu;

    private static final double TURRET_KP = 0.02;
    private static final double TURRET_KD = 0.002;
    private static final double TURRET_KF = 0.003;
    private static final double TURRET_MAX_POWER = 0.4;
    private static final double TURRET_DEADBAND = 0.6;
    private static final double TX_FILTER_ALPHA = 0.2;
    private static final double MAX_DELTA_POWER = 0.05;

    private double lastTx = 0, filteredTx = 0, lastYaw = 0, lastPower = 0;
    private long lastTime;

    public TurretSubsystem2(HardwareMap hw) {
        turret = hw.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hw.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(1);

        imu = hw.get(IMU.class,"imu");
        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(hubOrientation));

        lastTime = System.nanoTime();
        limelight.start();
    }

    public void update() {
        long now = System.nanoTime();
        double dt = (now-lastTime)/1e9;
        if (dt <= 0) return;
        lastTime = now;

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(AngleUnit.DEGREES);
        double yawRate = (yaw - lastYaw)/dt;
        lastYaw = yaw;

        limelight.updateRobotOrientation(yaw);

        LLResult result = limelight.getLatestResult();
        double power=0;

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            filteredTx = TX_FILTER_ALPHA*tx + (1-TX_FILTER_ALPHA)*filteredTx;
            double txRate = (filteredTx - lastTx)/dt;
            lastTx = filteredTx;

            if (Math.abs(filteredTx) > TURRET_DEADBAND) {
                power = -filteredTx*TURRET_KP - txRate*TURRET_KD - yawRate*TURRET_KF;
            }

            power = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER,power));
            power = Math.max(lastPower-MAX_DELTA_POWER, Math.min(lastPower+MAX_DELTA_POWER,power));
            lastPower = power;
        } else {
            lastPower = 0; power=0;
        }

        turret.setPower(power);
    }

    public void setPower(double power) {
        turret.setPower(power);
        lastPower = power;
    }

    public void stop() {
        turret.setPower(0);
    }
}
