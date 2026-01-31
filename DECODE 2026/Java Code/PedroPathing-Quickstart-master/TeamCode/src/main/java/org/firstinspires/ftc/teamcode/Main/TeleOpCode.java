package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FinalTeleOp")
public class TeleOpCode extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    // ----- Drive Motors -----
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // ----- Servos -----
    private DcMotor intakeMotor;

    private Servo kickServo;

    // ----- Shooter (SPARKmini acts as CRServo) -----
    private DcMotor shooterMotor;

    // ----- Sensors -----
    //private TouchSensor kickerLimit;
    //private ColorSensor colorSensor;

    // ----- Constants -----
    private final double GP1_DPAD_SPEED = 0.50;     // half speed for gp1 d-pad
    private final double GP2_DPAD_SPEED = 0.15;     // very low speed for gp2 precision
    private final double KICK_REST = -1.0;
    private final double KICK_FIRE = 0.4;

    // Shooter speeds chosen by gamepad2 (default = low)
    private double shooterSpeed = 0.88; // 1.0

    @Override
    public void init() {

        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Servos
        intakeMotor   = hardwareMap.get(DcMotor.class, "intakeMotor");
        kickServo     = hardwareMap.get(Servo.class,   "kickServo");

        // Shooter (SPARKmini)
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        // Sensors
        //kickerLimit  = hardwareMap.get(TouchSensor.class, "kickerSafety");
        //colorSensor  = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Motor direction (same as your working code)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Stop subsystems
        intakeMotor.setPower(0);

        // Set kicker to safe resting angle
        kickServo.setPosition(KICK_REST);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ======================================================
        //                 GAMEPAD 1 DPAD MOVEMENT
        // ======================================================
        if (gamepad1.dpad_up || gamepad1.dpad_down ||
                gamepad1.dpad_left || gamepad1.dpad_right) {

            double p = GP1_DPAD_SPEED;
            double fl=0, fr=0, bl=0, br=0;

            if (gamepad1.dpad_right)     { fl=p; fr=p; bl=p; br=p; } // up
            if (gamepad1.dpad_left)   { fl=-p; fr=-p; bl=-p; br=-p; }// down
            if (gamepad1.dpad_down)   { fl=-p; fr=p; bl=p; br=-p; } // left
            if (gamepad1.dpad_up)  { fl=p; fr=-p; bl=-p; br=p; }// right

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.addData("Drive Mode", "GP1 D-Pad (Half Speed)");
            telemetry.update();
            return;
        }

        // ======================================================
        //                 GAMEPAD 2 DPAD PRECISION
        // ======================================================
        if (gamepad2.dpad_up || gamepad2.dpad_down ||
                gamepad2.dpad_left || gamepad2.dpad_right) {

            double p = GP2_DPAD_SPEED;
            double fl=0, fr=0, bl=0, br=0;

            if (gamepad2.dpad_up)     { fl=p; fr=p; bl=p; br=p; }
            if (gamepad2.dpad_down)   { fl=-p; fr=-p; bl=-p; br=-p; }
            if (gamepad2.dpad_left)   { fl=-p; fr=p; bl=p; br=-p; }
            if (gamepad2.dpad_right)  { fl=p; fr=-p; bl=-p; br=p; }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.addData("Drive Mode", "GP2 Precision D-Pad");
            telemetry.update();
            return;
        }

        // ======================================================
        //                NORMAL MECANUM DRIVE (GP1)
        // ======================================================
        double forward  = gamepad1.left_stick_x;
        double strafe   =  -gamepad1.left_stick_y;
        double rotation =  gamepad1.right_stick_x;

        double[] drivePower = mecanum(forward, strafe, rotation);

        frontLeft.setPower(drivePower[0]);
        frontRight.setPower(drivePower[1]);
        backLeft.setPower(drivePower[2]);
        backRight.setPower(drivePower[3]);

        // ======================================================
        //              INTAKE + TRANSFER (GP1 RB)
        // ======================================================
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        } else if (gamepad1.left_bumper){
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        // ======================================================
        //               KICKER SERVO (GP1 A HOLD)
        // ======================================================
        if (gamepad1.a) {
            // protect from over-rotation
            kickServo.setPosition(KICK_FIRE);

        } else {
            kickServo.setPosition(KICK_REST);
        }

        // ======================================================
        //      SHOOTER SPEED SELECTION (GP2 A/B/X/Y)
        // ======================================================
        if (gamepad2.a) shooterSpeed = 0.88;
        if (gamepad2.b) shooterSpeed = 0.80;
        if (gamepad2.x) shooterSpeed = 0.95;
        if (gamepad2.y) shooterSpeed = 0.70;

        // ======================================================
        //           SHOOTER ACTIVATION (GP1 Right Trigger)
        // ======================================================
        if (gamepad1.right_trigger > 0.1) {
            shooterMotor.setPower(shooterSpeed);
        } else {
            shooterMotor.setPower(0);
        }

        // ======================================================
        //                 TELEMETRY
        // ======================================================
        telemetry.addData("Drive", "fwd=%.2f str=%.2f rot=%.2f", forward, strafe, rotation);
        telemetry.addData("Shooter Speed", shooterSpeed);
        //telemetry.addData("Color", "R:%d  G:%d  B:%d",
        //colorSensor.red(), colorSensor.green(), colorSensor.blue());
        //telemetry.addData("Kicker Safety", kickerLimit.isPressed() ? "Pressed" : "Free");
        telemetry.update();
    }


    // ---------------- MECANUM CALC ----------------
    private double[] mecanum(double f, double s, double r) {
        double fl = f + s + r;
        double fr = f - s - r;
        double bl = f - s + r;
        double br = f + s - r;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        return new double[]{ fl/max, fr/max, bl/max, br/max };
    }
}
