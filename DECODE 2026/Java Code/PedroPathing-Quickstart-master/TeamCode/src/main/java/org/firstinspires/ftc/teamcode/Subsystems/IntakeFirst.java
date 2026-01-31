package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeFirst {
    private final DcMotor intakeMotor;
    public TransferAndIntake(HardwareMap map){
        intakeMotor = map.get(DcMotor.class, "intakeMotor");
        intakeOff();

    }
    public void intakeOff(){
        intakeMotor.setPower(0);
    }

    public void intakeOn(){
        intakeMotor.setPower(-1);
    }
}
