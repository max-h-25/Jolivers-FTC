package org.firstinspires.ftc.teamcode.java;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FoidsGottaGo Backup", group = "StarterBot")
public class biobuzzprogramBackup extends OpMode {

    public static final String[] PRESET_NAMES = {"CLOSE", "MID", "FAR", "MAX"};
    private final double[] presetVelocity = {1250, 1450, 1650, 1900};

    public static final double LAUNCHER_MAX_VELOCITY = 2600;
    public static final double LAUNCHER_MIN_PRESET = 600;
    public static final double TRIM_STEP = 25;
    public static final double READY_RATIO = 0.96;
    public static final double HOLD_RATIO = 0.88;

    public static final double LAUNCHER_P = 40;
    public static final double LAUNCHER_I = 0;
    public static final double LAUNCHER_D = 0;
    public static final double LAUNCHER_F = 12.5;
    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double VOLTAGE_READ_SECONDS = 0.5;

    public static final double GATE_POWER = 0.5;
    public static final double GATE_MOVE_SECONDS = 0.6;

    public static final double SLOW_DRIVE_SCALE = 0.4;
    public static final double STICK_DEADZONE = 0.05;
    public static final double WRITE_EPSILON = 0.001;

    private static final String[] DRIVE_NAMES = {
            "left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive"};
    private static final DcMotorSimple.Direction[] DRIVE_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD,
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};

    private final DcMotor[] drive = new DcMotor[4];
    private final double[] drivePower = new double[4];
    private final double[] lastDrivePower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServoImplEx gate;
    private VoltageSensor battery;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double intakePower;
    private double lastIntakePower = Double.NaN;
    private double lastLauncherTarget = Double.NaN;
    private double gatePower;
    private double lastGatePower = Double.NaN;
    private double gateTravel = 0;
    private double launcherVelocity;
    private double batteryVoltage;

    private int presetIndex = 0;
    private boolean shooterOn = false;
    private boolean launcherReady = false;
    private boolean slowDrive = false;
    private boolean gateUp = false;

    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevY;
    private boolean prevA;
    private boolean prevX;
    private boolean prevRightBumper;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        for (int i = 0; i < 4; i++) {
            drive[i] = hardwareMap.get(DcMotor.class, DRIVE_NAMES[i]);
            drive[i].setDirection(DRIVE_DIRECTIONS[i]);
            drive[i].setZeroPowerBehavior(BRAKE);
        }

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        gate = hardwareMap.get(CRServoImplEx.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        battery = hardwareMap.voltageSensor.iterator().next();

        intake.setZeroPowerBehavior(BRAKE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        gate.setDirection(DcMotorSimple.Direction.REVERSE);
        gate.setPwmRange(new PwmControl.PwmRange(500, 2500));
        gate.setPower(0);
        lastGatePower = 0;

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyLauncherTuning();

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Put the gate DOWN before you press start");
        telemetry.addLine("RT = intake in   LT = intake out");
        telemetry.addLine("LB = gate up (feed)   B = gate down (block)");
        telemetry.addLine("RB = shooter on / off");
        telemetry.addLine("Dpad down / left / up / right = CLOSE / MID / FAR / MAX");
        telemetry.addLine("Y / A = raise / lower current shot speed");
        telemetry.addLine("X = slow drive on / off");
    }

    @Override
    public void start() {
        voltageTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        launcherVelocity = launcher.getVelocity();

        if (voltageTimer.seconds() >= VOLTAGE_READ_SECONDS) {
            batteryVoltage = battery.getVoltage();
            voltageTimer.reset();
        }

        handleButtons();

        mecanumDrive(
                deadzone(-gamepad1.left_stick_y),
                deadzone(gamepad1.left_stick_x),
                deadzone(gamepad1.right_stick_x),
                slowDrive ? SLOW_DRIVE_SCALE : 1.0);

        runIntake();
        runGate(dt);
        runShooter();

        double target = targetVelocity();
        telemetry.addData("Shot", "%s  (%.0f ticks/s)", PRESET_NAMES[presetIndex], target);
        telemetry.addData("Shooter", launcherStatus());
        telemetry.addData("Launcher speed", "%.0f / %.0f", launcherVelocity, shooterOn ? target : 0);
        telemetry.addData("Gate", gateStatus());
        telemetry.addData("Intake power", "%.2f", intakePower);
        telemetry.addData("Slow drive", slowDrive ? "ON" : "OFF");
        telemetry.addData("Battery", "%.1f V", batteryVoltage);
        telemetry.addData("Presets", "CLOSE %.0f  MID %.0f  FAR %.0f  MAX %.0f",
                presetVelocity[0], presetVelocity[1], presetVelocity[2], presetVelocity[3]);
    }

    @Override
    public void stop() {
        launcher.setVelocity(0);
        gate.setPower(0);
        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        for (DcMotor motor : drive) {
            motor.setPower(0);
        }
    }

    private void handleButtons() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean y = gamepad1.y;
        boolean a = gamepad1.a;
        boolean x = gamepad1.x;
        boolean rightBumper = gamepad1.right_bumper;

        if (down && !prevDpadDown) selectPreset(0);
        if (left && !prevDpadLeft) selectPreset(1);
        if (up && !prevDpadUp) selectPreset(2);
        if (right && !prevDpadRight) selectPreset(3);
        if (y && !prevY) trimPreset(TRIM_STEP);
        if (a && !prevA) trimPreset(-TRIM_STEP);
        if (x && !prevX) slowDrive = !slowDrive;

        if (rightBumper && !prevRightBumper) {
            shooterOn = !shooterOn;
            if (shooterOn) applyLauncherTuning();
        }

        if (gamepad1.left_bumper) {
            gateUp = true;
        } else if (gamepad1.b) {
            gateUp = false;
        }

        prevDpadUp = up;
        prevDpadDown = down;
        prevDpadLeft = left;
        prevDpadRight = right;
        prevY = y;
        prevA = a;
        prevX = x;
        prevRightBumper = rightBumper;
    }

    private void runIntake() {
        intakePower = Range.clip(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
        if (Math.abs(intakePower - lastIntakePower) < WRITE_EPSILON) return;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
        lastIntakePower = intakePower;
    }

    private void runGate(double dt) {
        if (gateUp && gateTravel < GATE_MOVE_SECONDS) {
            gatePower = GATE_POWER;
            gateTravel = Math.min(GATE_MOVE_SECONDS, gateTravel + dt);
        } else if (!gateUp && gateTravel > 0) {
            gatePower = -GATE_POWER;
            gateTravel = Math.max(0, gateTravel - dt);
        } else {
            gatePower = 0;
        }

        if (gatePower != lastGatePower) {
            gate.setPower(gatePower);
            lastGatePower = gatePower;
        }
    }

    private void runShooter() {
        double target = shooterOn ? targetVelocity() : 0;
        if (target != lastLauncherTarget) {
            launcher.setVelocity(target);
            lastLauncherTarget = target;
        }

        if (!shooterOn) {
            launcherReady = false;
            return;
        }

        boolean wasReady = launcherReady;
        if (launcherVelocity > target * READY_RATIO) {
            launcherReady = true;
        } else if (launcherVelocity < target * HOLD_RATIO) {
            launcherReady = false;
        }

        if (launcherReady && !wasReady) {
            gamepad1.rumble(200);
        }
    }

    private void selectPreset(int index) {
        presetIndex = index;
        gamepad1.rumbleBlips(index + 1);
    }

    private void trimPreset(double amount) {
        presetVelocity[presetIndex] = Range.clip(
                presetVelocity[presetIndex] + amount, LAUNCHER_MIN_PRESET, LAUNCHER_MAX_VELOCITY);
    }

    private double targetVelocity() {
        return presetVelocity[presetIndex];
    }

    private void applyLauncherTuning() {
        batteryVoltage = battery.getVoltage();
        double f = batteryVoltage > 1 ? LAUNCHER_F * NOMINAL_VOLTAGE / batteryVoltage : LAUNCHER_F;
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(LAUNCHER_P, LAUNCHER_I, LAUNCHER_D, f));
    }

    private String launcherStatus() {
        if (!shooterOn) return "OFF";
        return launcherReady ? "AT SPEED" : "SPINNING UP";
    }

    private String gateStatus() {
        if (gatePower > 0) return "GOING UP";
        if (gatePower < 0) return "GOING DOWN";
        return gateUp ? "UP (feeding)" : "DOWN (blocking)";
    }

    private double deadzone(double value) {
        return Math.abs(value) < STICK_DEADZONE ? 0 : value;
    }

    void mecanumDrive(double forward, double strafe, double rotate, double scale) {
        drivePower[0] = forward + strafe + rotate;
        drivePower[1] = forward - strafe - rotate;
        drivePower[2] = forward - strafe + rotate;
        drivePower[3] = forward + strafe - rotate;

        double max = 1.0;
        for (double p : drivePower) max = Math.max(max, Math.abs(p));
        double k = scale / max;

        for (int i = 0; i < 4; i++) {
            double power = drivePower[i] * k;
            if (Math.abs(power - lastDrivePower[i]) < WRITE_EPSILON) continue;
            drive[i].setPower(power);
            lastDrivePower[i] = power;
        }
    }
}
