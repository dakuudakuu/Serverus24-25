package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="SpecBucketAutonomous")
public class SpecBucketAutonomous extends LinearOpMode {

    //Controls Arm Pivot, Wrist Pivot, and Claw Opening/Closing
    public class Arm {
        private final Servo claw;
        private final Servo wrist;
        private final Servo arm;
        private final Servo backClaw;
        private final DcMotor backArm;

        private double armPos = ServoConfig.ARM_UP;
        private double clawPos = ServoConfig.CLAW_CLOSED;
        private double backClawPos = ServoConfig.CLAW_CLOSED;
        public int backArmTarget = 0;

        private PIDController controller;
        public double p = 0.003, i = 0, d = 0.0002, f = 0.005;

        public boolean usePID = true;

        //Arm Constructor
        public Arm(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "servo2");
            wrist = hardwareMap.get(Servo.class, "servo1");
            arm = hardwareMap.get(Servo.class, "servo0");

            wrist.setDirection(Servo.Direction.REVERSE);
            backArm = hardwareMap.get(DcMotor.class, "emotor2");
            backArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backClaw = hardwareMap.get(Servo.class, "servo3");

            controller = new PIDController(p, i, d);
        }

        //Loop that constantly sets servo positions
        public class ArmPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(clawPos);
                arm.setPosition(armPos);
                wrist.setPosition(ServoConfig.WRIST_DOWN);
                backClaw.setPosition(backClawPos);

                if(usePID) {
                    controller.setPID(p, i, d);
                    int backArmPos = backArm.getCurrentPosition();
                    double pid = controller.calculate(backArmPos, backArmTarget);
                    double power = pid + f;
                    backArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    backArm.setPower(power);
                } else {
                    backArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    backArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if(backArm.getCurrentPosition() > 500) {
                        backArm.setPower(-1);
                    } else {
                        backArm.setPower(0);
                    }
                }

                return opModeIsActive();
            }
        }
        public Action armPositions() {
            return new ArmPositions();
        }

        //Moves backArm Up
        public class BackArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                backArmTarget = 570;
                usePID = true;
                return false;
            }
        }
        public Action backArmUp() {
            return new BackArmUp();
        }

        //Moves backArm down
        public class backArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                backArmTarget = 0;
                usePID = true;
                return false;
            }
        }
        public Action backArmDown() {
            return new backArmDown();
        }

        //Hooks Specimen
        public class ArmHook implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                backArmTarget = 410;
                usePID = false;
                return false;
            }
        }
        public Action armHook() {
            return new ArmHook();
        }

        //Sets claw position variable to the "Closed" position variable
        public class BackGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                backClawPos = ServoConfig.CLAW_CLOSED;
                return false;
            }
        }
        public Action backGrab() {
            return new BackGrab();
        }

        //Sets claw position variable to the "Open" position variable
        public class BackRelease implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                backClawPos = ServoConfig.CLAW_OPEN;
                return false;
            }
        }
        public Action backRelease() {
            return new BackRelease();
        }

        //Sets arm position variables to the "Up" position
        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armPos = ServoConfig.ARM_UP;
                return false;
            }
        }
        public Action armUp() {
            return new ArmUp();
        }

        //Sets arm position variables to the "Down" position variables
        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armPos = ServoConfig.ARM_DOWN;
                return false;
            }
        }
        public Action armDown() {
            return new ArmDown();
        }

        //Sets arm position variables to the "Back" position variables
        public class ArmBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armPos = ServoConfig.ARM_MIDDLE;
                return false;
            }
        }
        public Action armBucket() {
            return new ArmBucket();
        }

        //Sets claw position variable to the "Closed" position variable
        public class Grab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawPos = ServoConfig.CLAW_CLOSED;
                return false;
            }
        }
        public Action grab() {
            return new Grab();
        }

        //Sets claw position variable to the "Open" position variable
        public class Release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawPos = ServoConfig.CLAW_OPEN;
                return false;
            }
        }
        public Action release() {
            return new Release();
        }
    }

    //Controls the positions of the vertical slides
    public class Lift {
        private final DcMotor slide0;
        private final DcMotor slide1;

        private int liftPosition = 0;

        //Lift Constructor
        public Lift(HardwareMap hardwareMap) {
            slide1 = hardwareMap.get(DcMotor.class, "emotor1");
            slide0 = hardwareMap.get(DcMotor.class, "emotor0");

            slide0.setDirection(DcMotor.Direction.FORWARD);
            slide0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slide1.setDirection(DcMotor.Direction.REVERSE);
            slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //Loop that sets vertical slide positions
        public class LiftPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slide0.setTargetPosition(liftPosition);
                slide0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(slide0.getCurrentPosition() - liftPosition) > 10) {
                    slide0.setPower(1);
                } else {
                    slide0.setPower(0);
                }
                slide1.setTargetPosition(liftPosition);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(slide1.getCurrentPosition() - liftPosition) > 10) {
                    slide1.setPower(1);
                } else {
                    slide1.setPower(0);
                }

                return opModeIsActive();
            }
        }
        public Action liftPositions() {
            return new LiftPositions();
        }

        //Sets vertical slide position variable to the "Up" position
        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftPosition = 1900;
                return false;
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        //Sets the vertical slide position variable to the "Down" position
        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftPosition = 0;
                return false;
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftMiddle implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftPosition = 500;
                return false;
            }
        }
        public Action liftMiddle() {
            return new LiftMiddle();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24.00, -63.00, Math.toRadians(90.00)));
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        waitForStart();

        //Runs when you press play
        if (opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(
                    lift.liftPositions(),
                    arm.armPositions(),
                    new SequentialAction(
                            new SleepAction(0.1),

                            //To bar
                            arm.backArmUp(),
                            arm.release(),
                            drive.actionBuilder(new Pose2d(-24.00, -63.00, Math.toRadians(90.00)))
                                    .strafeToLinearHeading(new Vector2d(-16, -33.5), Math.toRadians(90))
                                    .build(),

                            //Hook
                            arm.armHook(),
                            new SleepAction(0.3),
                            arm.backRelease(),
                            arm.backArmDown(),

                            drive.actionBuilder(new Pose2d(-16, -33.5, Math.toRadians(90)))
                                    .strafeToLinearHeading(new Vector2d(-52, -43.5), Math.toRadians(90))
                                    .build(),

                            arm.armDown(),
                            new SleepAction(0.8),
                            arm.grab(),
                            new SleepAction(0.2),
                            arm.armUp(),
                            new SleepAction(0.4),

                            drive.actionBuilder(new Pose2d(-52, -43.5, Math.toRadians(90.00)))
                                    .strafeToLinearHeading(new Vector2d(-51, -53), Math.toRadians(225))
                                    .build(),

                            lift.liftUp(),
                            new SleepAction(2),
                            arm.armDown(),
                            new SleepAction(0.5),
                            arm.release(),
                            new SleepAction(0.2),
                            arm.armUp(),
                            new SleepAction(0.4),
                            lift.liftDown(),
                            new SleepAction(0.5),

                            drive.actionBuilder(new Pose2d(-51, -53, Math.toRadians(225)))
                                    .strafeToLinearHeading(new Vector2d(-58, -42), Math.toRadians(90))
                                    .build(),

                            arm.armDown(),
                            new SleepAction(0.8),
                            arm.grab(),
                            new SleepAction(0.2),
                            arm.armUp(),
                            new SleepAction(0.4),

                            drive.actionBuilder(new Pose2d(-58, -42, Math.toRadians(90.00)))
                                    .strafeToLinearHeading(new Vector2d(-51, -53), Math.toRadians(225))
                                    .build(),

                            lift.liftUp(),
                            new SleepAction(2),
                            arm.armDown(),
                            new SleepAction(0.5),
                            arm.release(),
                            new SleepAction(0.2),
                            arm.armUp(),
                            new SleepAction(0.4),
                            lift.liftDown(),
                            new SleepAction(0.5),

                            drive.actionBuilder(new Pose2d(-51, -53, Math.toRadians(225)))
                                    .strafeToLinearHeading(new Vector2d(-59.5, -39.5), Math.toRadians(117))
                                    .build(),

                            arm.armDown(),
                            new SleepAction(0.8),
                            arm.grab(),
                            new SleepAction(0.5),

                            drive.actionBuilder(new Pose2d(-59.5, -39.5, Math.toRadians(117)))
                                            .turnTo(Math.toRadians(90.00))
                                            .build(),

                            arm.armUp(),
                            new SleepAction(0.4),

                            drive.actionBuilder(new Pose2d(-59.5, -39.5, Math.toRadians(90.00)))
                                    .strafeToLinearHeading(new Vector2d(-51, -53), Math.toRadians(225))
                                    .build(),

                            lift.liftUp(),
                            new SleepAction(2),
                            arm.armDown(),
                            new SleepAction(0.5),
                            arm.release(),
                            new SleepAction(0.2),
                            arm.armUp(),
                            new SleepAction(0.4),
                            lift.liftMiddle(),

                            drive.actionBuilder(new Pose2d(-51, -53, Math.toRadians(225)))
                                    .setTangent(Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(-40, -6), Math.toRadians(0))
                                    .strafeToLinearHeading(new Vector2d(-30, -6), Math.toRadians(0))
                                    .build(),

                            arm.armDown()
                    )
            ));
        }
    }
}