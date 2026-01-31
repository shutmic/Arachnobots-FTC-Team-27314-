package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferAndIntake {

    private DcMotor intakeMotor;

    public TransferAndIntake(HardwareMap hw) {
        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
    }

    // Control intake direction
    public void update(boolean reverse, boolean forward) {
        if (reverse) intakeMotor.setPower(-1);
        else if (forward) intakeMotor.setPower(1);
        else intakeMotor.setPower(0);
    }

    // Stop the motor
    public void stop() {
        intakeMotor.setPower(0);
    }
}
