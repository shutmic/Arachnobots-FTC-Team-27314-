package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferIntake {

    private final DcMotor transferMotor;

    public TransferIntake(HardwareMap map){

        transferMotor = map.get(DcMotor.class, "transferMotor");
        transferOff();
    }
    public void transferOn(){
        transferMotor.setPower(-1);
    }

    public void transferOff(){
        transferMotor.setPower(0);
    }

}
