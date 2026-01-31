package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(HardwareMap hw) {
        frontLeft = hw.get(DcMotorEx.class, "frontLeft");
        frontRight = hw.get(DcMotorEx.class, "frontRight");
        backLeft = hw.get(DcMotorEx.class, "backLeft");
        backRight = hw.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);
        backLeft.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);

        frontLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPowerAll(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stop() {
        setPowerAll(0,0,0,0);
    }

    public void update(com.qualcomm.robotcore.hardware.Gamepad gamepad) {
        double y = -gamepad.left_stick_y;
        double x = gamepad.left_stick_x * 1.1;
        double rx = gamepad.right_stick_x;

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; bl /= max; fr /= max; br /= max;
        }

        setPowerAll(fl, fr, bl, br);
    }
}
