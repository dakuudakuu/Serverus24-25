package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="MainTeleop", group = "Linear OpMode")
public class MainTeleop extends LinearOpMode {
    public static boolean clawButtonState = false;
    public static boolean clawClosed = true;

    public DcMotor slide1;
    public DcMotor slide0;

    public Servo claw;
    public Servo wrist;
    public Servo arm;
    public Servo backClaw;

    public DcMotor motor0;
    public DcMotor motor1;
    public DcMotor motor2;
    public DcMotor motor3;

    public DcMotor backArm;

    private PIDController controller;
    public static double p = 0.003, i = 0, d = 0.0002, f = 0.005;
    public static int backArmTarget = 0;

    int armPosition = 1;

    @Override public void runOpMode() {
        config();
        waitForStart();
        while (opModeIsActive()) {
            moveBackArm();
            armPivot();
            slideExtension();
            claw();
            drive();
        }
    }

    public void moveBackArm() {
        if(gamepad2.dpad_up) {
            backArmTarget = 575;
        } else if (gamepad2.dpad_down) {
            backArmTarget = 0;
        }

        controller.setPID(p, i, d);
        int backArmPos = backArm.getCurrentPosition();
        double pid = controller.calculate(backArmPos, backArmTarget);
        double power = pid + f;
        backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backArm.setPower(power);

        telemetry.addData("pos", backArmPos);
        telemetry.addData("target", backArmTarget);
        telemetry.update();
    }

    public void slideExtension() {
        if(gamepad2.right_bumper) {
            slide0.setPower(1);
            slide1.setPower(1);
        } else if (gamepad2.left_bumper) {
            slide0.setPower(-1);
            slide1.setPower(-1);
        } else {
            slide0.setPower(0);
            slide1.setPower(0);
        }
    }

    public void claw() {
        if (gamepad2.a & gamepad2.a != clawButtonState) {
            clawClosed = !clawClosed;
        }
        clawButtonState = gamepad2.a;

        if(clawClosed) {
            claw.setPosition(ServoConfig.CLAW_CLOSED);
            backClaw.setPosition(ServoConfig.CLAW_CLOSED);
        } else {
            claw.setPosition(ServoConfig.CLAW_OPEN);
            backClaw.setPosition(ServoConfig.CLAW_OPEN);
        }
    }

    public void armPivot() {
        if(gamepad2.x) {
            armPosition = 0;
        } else if(gamepad2.y) {
            armPosition = 1;
        } else if(gamepad2.b) {
            armPosition = 2;
        } else if(gamepad2.dpad_right) {
            armPosition = 3;
        }

        switch(armPosition) {
            case 1:
                arm.setPosition(ServoConfig.ARM_UP);
                wrist.setPosition(ServoConfig.WRIST_DOWN);
                break;
            case 2:
                arm.setPosition(ServoConfig.ARM_DOWN);
                wrist.setPosition(ServoConfig.WRIST_UP);
                break;
            case 0:
                arm.setPosition(ServoConfig.ARM_MIDDLE);
                wrist.setPosition(ServoConfig.WRIST_MIDDLE);
                break;
            case 3:
                arm.setPosition(ServoConfig.ARM_DOWN);
                wrist.setPosition(ServoConfig.WRIST_DOWN);
                break;
        }
    }

    public void drive() {
        double speed = 0.8;
        if(gamepad1.left_bumper) {
            speed = 0.3;
        }

        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        motor0.setPower((axial + lateral + yaw) * speed);
        motor1.setPower((axial - lateral - yaw) * speed);
        motor2.setPower((axial + lateral - yaw) * speed);
        motor3.setPower((axial - lateral + yaw) * speed);
    }

    public void config() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");

        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide1 = hardwareMap.get(DcMotor.class, "emotor1");
        slide0 = hardwareMap.get(DcMotor.class, "emotor0");

        slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide0.setDirection(DcMotor.Direction.FORWARD);
        slide0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setDirection(DcMotor.Direction.REVERSE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw = hardwareMap.get(Servo.class, "servo2");
        wrist = hardwareMap.get(Servo.class, "servo1");
        arm = hardwareMap.get(Servo.class, "servo0");

        wrist.setDirection(Servo.Direction.REVERSE);

        backArm = hardwareMap.get(DcMotor.class, "emotor2");
        backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backClaw = hardwareMap.get(Servo.class, "servo3");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
}