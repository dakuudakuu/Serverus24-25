package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous(name="SpecAutonomous")
public class SpecAutonomous extends LinearOpMode {

    //Controls Arm Pivot and Claw Opening/Closing
    public class Arm {
        private final Servo claw;
        private final Servo frontArm;
        private final DcMotor arm;

        private double clawPos = ServoConfig.CLAW_CLOSED;
        private PIDController controller;
        public double p = 0.003, i = 0, d = 0.0002, f = 0.005;
        public int backArmTarget = 0;

        public boolean usePID = true;

        //Arm Constructor
        public Arm(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "servo3");
            arm = hardwareMap.get(DcMotor.class, "emotor2");
            frontArm = hardwareMap.get(Servo.class, "servo0");
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            controller = new PIDController(p, i, d);
        }

        //Loop that constantly sets positions
        public class ArmPositions implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(clawPos);
                frontArm.setPosition(1);

                if(usePID) {
                    controller.setPID(p, i, d);
                    int backArmPos = arm.getCurrentPosition();
                    double pid = controller.calculate(backArmPos, backArmTarget);
                    double power = pid + f;
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    arm.setPower(power);
                } else {
                    arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if(arm.getCurrentPosition() > 500) {
                        arm.setPower(-1);
                    } else {
                        arm.setPower(0);
                    }
                }

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
                backArmTarget = 570;
                usePID = true;
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
                backArmTarget = 0;
                usePID = true;
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
                backArmTarget = 410;
                usePID = false;
                return false;
            }
        }

        public Action armHook() {
            return new ArmHook();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24.00, -63, Math.toRadians(90.00)));
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        //Runs when you press play
        if (opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(
                    arm.armPositions(),
                    new SequentialAction(
                            new SleepAction(0.1),

                            //To bar
                            arm.armUp(),
                            drive.actionBuilder(new Pose2d(24, -63, Math.toRadians(90.00)))
                                    .strafeToConstantHeading(new Vector2d(-9, -33))
                                    .build(),

                            //Hook
                            arm.armHook(),
                            new SleepAction(0.3),
                            arm.release(),
                            arm.armDown(),

                            //Push
                            drive.actionBuilder(new Pose2d(-9, -33, Math.toRadians(90.00)))
                                    .strafeToConstantHeading(new Vector2d(36, -38), null, new ProfileAccelConstraint(-40, 40))
                                    .setTangent(90)
                                    .splineToConstantHeading(new Vector2d(46, -12), Math.toRadians(0))
                                    .build(),
                            drive.actionBuilder(new Pose2d(46, -12, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(46, -54), null, new ProfileAccelConstraint(-40, 40))
                                    .strafeToConstantHeading(new Vector2d(46, -17), null, new ProfileAccelConstraint(-40, 40))
                                    .build(),
                            drive.actionBuilder(new Pose2d(46, -17, Math.toRadians(90)))
                                    .splineToConstantHeading(new Vector2d(56, -12), Math.toRadians(0))
                                    .build(),
                            drive.actionBuilder(new Pose2d(56, -12, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(56, -54), null, new ProfileAccelConstraint(-40, 40))
                                    .build(),

                            //Drive to Wall
                            drive.actionBuilder(new Pose2d(56, -54, Math.toRadians(90)))
                                    .setTangent(Math.toRadians(90))
                                    .splineToConstantHeading(new Vector2d(40, -63.5), Math.toRadians(-90), null, new ProfileAccelConstraint(-20, 40))
                                    .build(),
                            new SleepAction(0.1),

                            //Grab the sample
                            arm.grab(),
                            new SleepAction(0.3),
                            arm.armUp(),

                            //Go to bar
                            drive.actionBuilder(new Pose2d(40, -63.5, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(-6, -32.75))
                                    .build(),

                            //Hook specimen
                            arm.armHook(),
                            new SleepAction(0.3),
                            arm.release(),
                            arm.armDown(),

                            //Drive to Wall
                            drive.actionBuilder(new Pose2d(-6, -32.75, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(36, -63.25), null, new ProfileAccelConstraint(-30, 40))
                                    .build(),
                            new SleepAction(0.1),

                            //Grab the sample
                            arm.grab(),
                            new SleepAction(0.3),
                            arm.armUp(),

                            //Go to bar
                            drive.actionBuilder(new Pose2d(36, -63.25, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(-3, -32.5))
                                    .build(),

                            //Hook specimen
                            arm.armHook(),
                            new SleepAction(0.3),
                            arm.release(),
                            arm.armDown(),

                            //Drive to Wall
                            drive.actionBuilder(new Pose2d(-3, -32.5, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(36, -63.25), null, new ProfileAccelConstraint(-30, 40))
                                    .build(),
                            new SleepAction(0.1),

                            //Grab the sample
                            arm.grab(),
                            new SleepAction(0.3),
                            arm.armUp(),

                            //Go to bar
                            drive.actionBuilder(new Pose2d(36, -63.25, Math.toRadians(90)))
                                    .strafeToConstantHeading(new Vector2d(0, -32.25))
                                    .build(),

                            //Hook specimen
                            arm.armDown(),
                            new SleepAction(0.3),
                            arm.release()
                    )
            ));
        }
    }
}