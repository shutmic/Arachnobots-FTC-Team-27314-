package org.firstinspires.ftc.teamcode.RobotTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Limelight Test")
public class LL extends OpMode {

    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1); // make sure this pipeline actually exists

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(hubOrientation));

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        // Update robot yaw for correct botpose
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botPose = result.getBotpose();

            telemetry.addData("Has Target", true);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());

            if (botPose != null) {
                telemetry.addData("Bot X", botPose.getPosition().x);
                telemetry.addData("Bot Y", botPose.getPosition().y);
                telemetry.addData("Bot Z", botPose.getPosition().z);
                telemetry.addData("Bot Yaw", botPose.getOrientation().getYaw());
            }

        } else {
            telemetry.addData("Has Target", false);
        }

        telemetry.update();
    }
}
