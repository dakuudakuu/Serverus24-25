package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Disabled
@Autonomous(name="MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    //Controls Arm Pivot, Wrist Pivot, and Claw Opening/Closing
    public class Arm {
        private final Servo claw;
        private final Servo wrist;
        private final Servo arm;
        private final boolean isdumb = true;

        private double armPos = ServoConfig.ARM_UP;
        private double clawPos = ServoConfig.CLAW_CLOSED;
        private double wristPos = ServoConfig.WRIST_MIDDLE;

        //Arm Constructor
        public Arm(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "servo2");
            wrist = hardwareMap.get(Servo.class, "servo1");
            arm = hardwareMap.get(Servo.class, "servo0");

            wrist.setDirection(Servo.Direction.REVERSE);
        }

        //Loop that constantly sets servo positions
        public class ArmPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(clawPos);
                wrist.setPosition(wristPos);
                arm.setPosition(armPos);
                return opModeIsActive();
            }
        }

        public Action armPositions() {
            return new ArmPositions();
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

        //Sets claw position variable to the "Open" position variable
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

        //Sets claw position variable to the "Open" position variable
        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armPos = ServoConfig.ARM_MIDDLE;
                return false;
            }
        }

        public Action armDown() {
            return new ArmDown();
        }

        //Sets claw position variable to the "Open" position variable
        public class ArmHook implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                armPos = ServoConfig.ARM_HOOK;
                return false;
            }
        }

        public Action armHook() {
            return new ArmHook();
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
                if (Math.abs(slide0.getCurrentPosition() - liftPosition) > 10) {
                    slide0.setPower(1);
                } else {
                    slide0.setPower(0);
                }
                slide1.setTargetPosition(liftPosition);
                slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (Math.abs(slide1.getCurrentPosition() - liftPosition) > 10) {
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
                liftPosition = 700;
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

        //Sets the vertical slide position variable to the "Down" position
        public class Hook implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                liftPosition = 400;
                return false;
            }
        }

        public Action hook() {
            return new Hook();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24.00, -63, Math.toRadians(90.00)));
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        waitForStart();

        //Runs when you press play
        if (opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(
                    lift.liftPositions(),
                    arm.armPositions(),
                    new SequentialAction(
                            //Lift slides and drive to top bar
                            lift.liftUp(),
                            drive.actionBuilder(new Pose2d(24.00, -63, Math.toRadians(90.00)))
                                    .strafeToConstantHeading(new Vector2d(9, -37))
                                    .build(),

                            //Hook Specimen
                            arm.armHook(),
                            new SleepAction(0.3),
                            lift.hook(),
                            new SleepAction(0.5),
                            arm.release(),
                            arm.armUp(),

                            //Bring the slides down and push pre-placed samples to the observation zone
                            lift.liftDown(),
                            drive.actionBuilder(new Pose2d(9, -37, Math.toRadians(90.00)))
                                    .setTangent(Math.toRadians(-90))
                                    .strafeToConstantHeading(new Vector2d(36, -38))
                                    .strafeToConstantHeading(new Vector2d(36, -10))
                                    .setTangent(Math.toRadians(0.00))
                                    .splineToConstantHeading(new Vector2d(46, -20), Math.toRadians(-90.00), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 30))
                                    .strafeToConstantHeading(new Vector2d(46, -54))
                                    .strafeToConstantHeading(new Vector2d(46, -10))
                                    .setTangent(Math.toRadians(0.00))
                                    .splineToConstantHeading(new Vector2d(56, -20), Math.toRadians(-90.00), new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 30))
                                    .strafeToConstantHeading(new Vector2d(56, -54))
                                    .strafeToConstantHeading(new Vector2d(48, -35))
                                    .turnTo(Math.toRadians(-90.00))
                                    .strafeToConstantHeading(new Vector2d(46.5, -53.25))
                                    .build(),

                            //Grab the sample
                            arm.armDown(),
                            new SleepAction(0.5),
                            arm.grab(),
                            new SleepAction(0.3),
                            arm.armUp(),

                            //Bring the slides up and drive to the top bar
                            lift.liftUp(),
                            drive.actionBuilder(new Pose2d(46.5, -53.25, Math.toRadians(-90.00)))
                                    .strafeToLinearHeading(new Vector2d(6, -37), Math.toRadians(90.00))
                                    .build(),

                            //Hook specimen
                            arm.armHook(),
                            new SleepAction(0.3),
                            lift.hook(),
                            new SleepAction(0.5),
                            arm.release(),
                            arm.armUp(),

                            //Drive to 2nd Grab
                            lift.liftDown(),
                            drive.actionBuilder(new Pose2d(6, -37, Math.toRadians(90.00)))
                                    .strafeToLinearHeading(new Vector2d(46.5, -53.25), Math.toRadians(-90.00))
                                    .build(),

                            //Grab the sample
                            arm.armDown(),
                            new SleepAction(0.5),
                            arm.grab(),
                            new SleepAction(0.3),
                            arm.armUp(),

                            //Bring the slides up and drive to top bar
                            lift.liftUp(),
                            drive.actionBuilder(new Pose2d(46.5, -53.25, Math.toRadians(-90.00)))
                                    .strafeToLinearHeading(new Vector2d(3, -37), Math.toRadians(90.00))
                                    .build(),

                            //hook specimen
                            arm.armHook(),
                            new SleepAction(0.3),
                            lift.hook(),
                            new SleepAction(0.5),
                            arm.release(),
                            arm.armUp(),

                            //Park
                            lift.liftDown(),
                            drive.actionBuilder(new Pose2d(3, -37, Math.toRadians(90)))
                                    .strafeToLinearHeading(new Vector2d(46.5, -53.25), Math.toRadians(90.00))
                                    .build()
                    )
            ));
        }
    }
}