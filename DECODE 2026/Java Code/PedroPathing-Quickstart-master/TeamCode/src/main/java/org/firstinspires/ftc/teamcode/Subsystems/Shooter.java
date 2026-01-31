package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Shooter")
public class Shooter extends OpMode {
    DcMotor shooterMotor;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            shooterMotor.setPower(1);
        } else if (gamepad1.dpad_left) {
            shooterMotor.setPower(0.1);
        } else if (gamepad1.dpad_right) {
            shooterMotor.setPower(0.3);
        }else if(gamepad1.dpad_down){
            shooterMotor.setPower(0.6);
        }else {
            shooterMotor.setPower(0);
        }
    }
}
