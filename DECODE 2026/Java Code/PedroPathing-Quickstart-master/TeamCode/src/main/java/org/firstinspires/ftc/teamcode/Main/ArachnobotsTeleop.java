package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

@TeleOp(name="Full Robot TeleOp (Updated)")
public class ArachnobotsTeleop extends OpMode {

    // ================= HARDWARE =================
    private MecanumDrive drive;
    private TurretShooter shooter;
    private TurretAim turretAim;
    private IntakeSystem intake;

    private Limelight3A limelight;
    private Servo scanServo;

    // ================= SERVO SCAN =================
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    private double scanAngle = 90;
    private double scanDir = 1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        scanServo = hardwareMap.get(Servo.class, "servoMotor");

        drive = new MecanumDrive(hardwareMap);
        shooter = new TurretShooter(hardwareMap, limelight);
        turretAim = new TurretAim(hardwareMap, limelight);
        intake = new IntakeSystem(hardwareMap);

        limelight.start();

        telemetry.addLine("TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        turretAim.start();
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        intake.update(gamepad1.left_bumper, gamepad1.right_bumper);

        LLResult result = limelight.getLatestResult();
        boolean hasTarget = result != null && result.isValid();

        // ================= SERVO + TURRET =================
        if (hasTarget) {
            // Lock servo when target exists
            // TurretAim handles fine alignment
        } else {
            scanServo();
        }

        turretAim.update();
        shooter.update(gamepad1.right_trigger > 0.2);

        telemetry.addData("Target", hasTarget);
        telemetry.addData("Servo Pos", scanServo.getPosition());
        telemetry.addData("Shooter Vel", shooter.getCurrentVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        shooter.stop();
        turretAim.stop();
        intake.stop();
    }

    // ================= SERVO SCAN =================
    private void scanServo() {
        scanAngle += scanDir * 2;

        if (scanAngle >= 180) {
            scanAngle = 180;
            scanDir = -1;
        } else if (scanAngle <= 0) {
            scanAngle = 0;
            scanDir = 1;
        }

        double pos = SERVO_MIN + (scanAngle / 180.0) * (SERVO_MAX - SERVO_MIN);
        scanServo.setPosition(pos);
    }

    // ================= MECANUM DRIVE =================
    public static class MecanumDrive {
        DcMotorEx fl, fr, bl, br;

        public MecanumDrive(HardwareMap hw) {
            fl = hw.get(DcMotorEx.class, "frontLeft");
            fr = hw.get(DcMotorEx.class, "frontRight");
            bl = hw.get(DcMotorEx.class, "backLeft");
            br = hw.get(DcMotorEx.class, "backRight");

            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);
        }

        public void update(Gamepad g) {
            double y = -g.left_stick_y;
            double x = g.left_stick_x * 1.1;
            double rx = g.right_stick_x;

            double flp = y + x + rx;
            double blp = y - x + rx;
            double frp = y - x - rx;
            double brp = y + x - rx;

            double max = Math.max(Math.abs(flp),
                    Math.max(Math.abs(blp), Math.max(Math.abs(frp), Math.abs(brp))));

            if (max > 1) {
                flp /= max; blp /= max; frp /= max; brp /= max;
            }

            fl.setPower(flp);
            bl.setPower(blp);
            fr.setPower(frp);
            br.setPower(brp);
        }

        public void stop() {
            fl.setPower(0); fr.setPower(0);
            bl.setPower(0); br.setPower(0);
        }
    }

    // ================= SHOOTER =================
    public static class TurretShooter {
        DcMotorEx motor;
        Limelight3A limelight;

        static final double MAX_RPM = 6000;
        static final double TICKS_PER_REV = 28;
        static final double MAX_TPS = (MAX_RPM / 60.0) * TICKS_PER_REV;

        static final double kP = 0.002, kD = 0.0001;
        static final double kF = 32767.0 / MAX_TPS;
        static final double RAMP = 300;

        private double targetVelocity = 0;
        private double currentVelocity = 0;
        private double lastValidTarget = 0;

        private final double[][] table = {
                {0.5, 1500},
                {1.0, 4000},
                {2.0, 6000}
        };

        public TurretShooter(HardwareMap hw, Limelight3A ll) {
            motor = hw.get(DcMotorEx.class, "shooterMotor");
            limelight = ll;

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocityPIDFCoefficients(kP, 0, kD, kF);
        }

        public void update(boolean fire) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()
                    && result.getFiducialResults().size() > 0) {

                Pose3D pose = result.getFiducialResults().get(0).getRobotPoseTargetSpace();
                double d = Math.hypot(pose.getPosition().x, pose.getPosition().z);
                lastValidTarget = lookup(d);
            }

            targetVelocity = lastValidTarget;

            if (fire) {
                currentVelocity += Math.signum(targetVelocity - currentVelocity) * RAMP;
                currentVelocity = Math.min(currentVelocity, targetVelocity);
            } else {
                currentVelocity = Math.max(0, currentVelocity - RAMP);
            }

            motor.setVelocity(currentVelocity);
        }

        private double lookup(double d) {
            if (d <= table[0][0]) return table[0][1];
            if (d >= table[2][0]) return table[2][1];

            for (int i = 0; i < table.length - 1; i++) {
                if (d >= table[i][0] && d <= table[i + 1][0]) {
                    double t = (d - table[i][0]) /
                            (table[i + 1][0] - table[i][0]);
                    return table[i][1] + t * (table[i + 1][1] - table[i][1]);
                }
            }
            return table[2][1];
        }

        public double getCurrentVelocity() {
            return currentVelocity;
        }

        public void stop() {
            motor.setPower(0);
        }
    }

    // ================= TURRET AIM =================
    public static class TurretAim {
        DcMotor turret;
        Limelight3A limelight;
        IMU imu;

        static final double KP = 0.02;
        static final double KD = 0.002;
        static final double KF = 0.003;
        static final double MAX = 0.4;
        static final double DEAD = 0.6;

        double lastTx = 0;
        double lastYaw = 0;
        double lastPower = 0;
        long lastTime;

        public TurretAim(HardwareMap hw, Limelight3A ll) {
            turret = hw.get(DcMotor.class, "turret");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            limelight = ll;
            limelight.pipelineSwitch(1);

            imu = hw.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

            lastTime = System.nanoTime();
        }

        public void start() {}

        public void update() {
            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            if (dt <= 0 || dt > 0.1) dt = 0.02;
            lastTime = now;

            double yaw = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);
            double yawRate = (yaw - lastYaw) / dt;
            lastYaw = yaw;

            limelight.updateRobotOrientation(yaw);

            LLResult r = limelight.getLatestResult();
            double power = 0;

            if (r != null && r.isValid()) {
                double tx = r.getTx();
                double txRate = (tx - lastTx) / dt;
                lastTx = tx;

                if (Math.abs(tx) > DEAD) {
                    power = -tx * KP - txRate * KD - yawRate * KF;
                }
            }

            power = Math.max(-MAX, Math.min(MAX, power));
            power = Math.max(lastPower - 0.05, Math.min(lastPower + 0.05, power));
            lastPower = power;

            turret.setPower(power);
        }

        public void stop() {
            turret.setPower(0);
        }
    }

    // ================= INTAKE =================
    public static class IntakeSystem {
        DcMotor intake;

        public IntakeSystem(HardwareMap hw) {
            intake = hw.get(DcMotor.class, "intakeMotor");
        }

        public void update(boolean reverse, boolean forward) {
            if (reverse) intake.setPower(-1);
            else if (forward) intake.setPower(1);
            else intake.setPower(0);
        }

        public void stop() {
            intake.setPower(0);
        }
    }
}
