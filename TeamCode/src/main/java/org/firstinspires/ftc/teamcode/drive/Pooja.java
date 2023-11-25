package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */

@Config
public class Pooja extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(10, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(20, 0, 1);

    public static double LATERAL_MULTIPLIER = 1.588;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    // Hardware Instantiation
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public double coarseTurnConstant = 0.7;
    public double fineTurnConstant = 0.2;

    public double brakeTolerance = 5.0;

    private DcMotorEx collectorBottom, collectorTop, dropperFront, dropperBack;
    private PIDController collectorExtensionController, collectorRetractionController, depositExtensionController, depositRetractionController;
    public static double maxCollectorExtensionPower, maxCollectorRetractionPower, maxDepositExtensionPower, maxDepositRetractionPower;
    private Servo coneStabilizer, clawRotate, claw, FBLeft, FBRight;
    private DistanceSensor collectorSensor; // depositSensorDistance;
//    private ColorSensor depositSensorColor;

    // Mechanism Constants
    public static double FBLeftIntakePos = 0.59;
    public static double FBRightIntakePos = 0.39;

    public static double FBLeftIntakeHigh = 0.58; //0.59
    public static double FBRightIntakeHigh = 0.42; //0.41

    public static double FBLeftIntakeTele = 0.58; //0.59
    public static double FBRightIntakeTele = 0.42; //0.41

    public static double FBLeftTransferPos = 0.07;
    public static double FBRightTransferPos = 0.93;

    public static double FBLeftRestPos = 0.17;
    public static double FBRightRestPos = 0.83;

    public static double FBLeftLowJunctionPos = 0.2;
    public static double FBRightLowJunctionPos = 0.8;

    public static double FBLeftGroundJunctionPos = 0.55;
    public static double FBRightGroundJunctionPos = 0.45;

    public static double FBLeftGroundJunctionHighPos = 0.42;
    public static double FBRightGroundJunctionHighPos = 0.58;

    public static double FBLeftInitPos = 0.03; //0.04
    public static double FBRightInitPos = 0.97; //0.96

    public static double FBLeftAfterInitPos = 0.1; //0.04
    public static double FBRightAfterInitPos = 0.9; //0.96

    public static double FBLeftTransferHighPos = 0.20; // 0.17
    public static double FBRightTransferHighPos = 0.80; // 0.83

    public static double FBLeftTransferLowPos = 0.03; // 0.07
    public static double FBRightTransferLowPos = 0.97; // 0.93

    public static double FBLeftIntakeStack5PosTele = 0.47;
    public static double FBRightIntakeStack5PosTele = 0.53;

    public static double FBLeftIntakeStack4PosTele = 0.50;
    public static double FBRightIntakeStack4PosTele = 0.50;

    public static double FBLeftIntakeStack3PosTele = 0.53;
    public static double FBRightIntakeStack3PosTele = 0.47;

    public static double FBLeftIntakeStack2PosTele = 0.57;
    public static double FBRightIntakeStack2PosTele = 0.43;

    public static double FBLeftGroundRaisedPos = 0.57;
    public static double FBRightGroundRaisedPos = 0.43;

    public static double openClawPos = 0.17;
    public static double partiallyOpenClawPos = 0.17;
    public static double closeClawAutoPos = 0.02;
    public static double closeClawTelePos = 0.02;
    public static double clawRotateToTransferPos = 0.77;
    public static double clawRotateToIntakePosLeftAuto = 0.14;
    public static double clawRotateToIntakePosRightAuto = 0.06;
    public static double clawRotateToIntakePosTeleOp = 0.1;

    public static double coneStabilizerDisengagedPos = 0.3;
    public static double coneStabilizerEngagedPos = 0.13;
    public static double coneStabilizerDisengageMultiplier = 0.95;

    public static int dropperSlidesHighPos = 870;
    public static int dropperSlidesMidPos = 520;
    private double collectorEncOffset = 0;

    // Auto Constants
    public static double openClawAutoPos = 0.03;

    public static double collectConesCyclesPos = 300;

    public static double maxAllowableBrakeDisplacementError = 0.8;
    public static double maxAllowableBrakeHeadingError = 0.8;

    public static double FBLeftIntakeStack5Pos = 0.48;
    public static double FBRightIntakeStack5Pos = 0.53;
    public static double collectorSlidesStack5 = 532;

    public static double FBLeftIntakeStack4Pos = 0.5;
    public static double FBRightIntakeStack4Pos = 0.5;
    public static double collectorSlidesStack4 = 532;

    public static double FBLeftIntakeStack3Pos = 0.52;
    public static double FBRightIntakeStack3Pos = 0.48;
    public static double collectorSlidesStack3 = 520;

    public static double FBLeftIntakeStack2Pos = 0.55; //0.56
    public static double FBRightIntakeStack2Pos = 0.45; //0.44
    public static double collectorSlidesStack2 = 527;

    public static double FBLeftIntakeStack1Pos = 0.58; //0.58
    public static double FBRightIntakeStack1Pos = 0.42; //0.42
    public static double collectorSlidesStack1 = 527;

    public static double dropperSlidesInitPos = 40;
    public static double maxAllowableDepositError = 1; // 10
    public static double maxAllowableCollectorError = 40; // 10
    public static double dropperSlidesHighPosAuto = 900; // 905
    public static double dropperSlidesRetractedPos = 50;
    private double dropperSlidesSlip = 0;
    private double collectorSlidesSlip = 0;
    public static double extensionPidThreshold = 500;
    public static double collectableConeDistance = 4;
    public static double distanceSensorTimeout = 1000;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public Pooja(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
//        imu.initialize(parameters);

        // Hardware Map Init
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        maxCollectorExtensionPower = 0.76;
        maxCollectorRetractionPower = 0.7;
        maxDepositExtensionPower = 1.0;
        maxDepositRetractionPower = 0.6;

        collectorBottom = hardwareMap.get(DcMotorEx.class, "collectorBottom");
        collectorTop = hardwareMap.get(DcMotorEx.class, "collectorTop");
//        collectorExtensionController = new PIDController(0.0045, 0.0, 0.0, maxCollectorExtensionPower);
        collectorExtensionController = new PIDController(0.006, 0.0, 0.0001, maxCollectorExtensionPower);
        collectorRetractionController = new PIDController(0.006, 0.0, 0.0001, maxCollectorRetractionPower);
        dropperBack = hardwareMap.get(DcMotorEx.class, "dropperBack");
        dropperFront = hardwareMap.get(DcMotorEx.class, "dropperFront");
        depositExtensionController = new PIDController(0.022, 0.0, 0.0002, maxDepositExtensionPower);
        depositRetractionController = new PIDController(0.022, 0.0, 0.0002, maxDepositRetractionPower);


        coneStabilizer = hardwareMap.get(Servo.class, "coneStabilizer");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        claw = hardwareMap.get(Servo.class, "claw");
        FBLeft = hardwareMap.get(Servo.class, "FBLeft");
        FBRight = hardwareMap.get(Servo.class, "FBRight");

        collectorSensor = hardwareMap.get(DistanceSensor.class, "collectorSensor");
//        depositSensorColor = hardwareMap.get(ColorSensor.class, "depositSensor");
//        depositSensorDistance = hardwareMap.get(DistanceSensor.class, "depositSensor");

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
         setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );

        // Init Mechanism Positions
        FBRight.setPosition(FBRightInitPos);
        FBLeft.setPosition(FBLeftInitPos);
        claw.setPosition(openClawPos);
        clawRotate.setPosition(clawRotateToIntakePosTeleOp);
        coneStabilizer.setPosition(coneStabilizerDisengagedPos); // TODO: change this back to engaged position

        collectorTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collectorBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectorBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dropperBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dropperBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dropperBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        disengageConeStabilizer();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (dropperSlidesInitPos - dropperBack.getCurrentPosition() > maxAllowableDepositError) {
            double targetPower = depositExtensionController.calculateUsingEncoder(dropperSlidesInitPos, dropperBack.getCurrentPosition());
            dropperBack.setPower(targetPower);
            dropperFront.setPower(targetPower);

            if (timer.seconds() > 3) {
                break;
            }
        }

        dropperFront.setPower(0.1);
        dropperBack.setPower(0.1);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void correctPosition(Pose2d correctionPosition) {
        Trajectory correctPosition = trajectoryBuilder(this.getPoseEstimate())
                .lineToLinearHeading(correctionPosition)
                .build();

        this.followTrajectory(correctPosition);
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    // Mechanism Functions
    public void FBToIntakeTele(int stackLevelIntake, int extensionRange) {
        double leftPos = FBLeftIntakePos;
        double rightPos = FBRightIntakePos;

        if (stackLevelIntake == 5) {
//            FBLeft.setPosition(FBLeftIntakeStack5Pos);
//            FBRight.setPosition(FBRightIntakeStack5Pos);
            leftPos = FBLeftIntakeStack5Pos;
            rightPos = FBRightIntakeStack5Pos;
        }
        if (stackLevelIntake == 4) {
//            FBLeft.setPosition(FBLeftIntakeStack4Pos);
//            FBRight.setPosition(FBRightIntakeStack4Pos);
            leftPos = FBLeftIntakeStack4Pos;
            rightPos = FBRightIntakeStack4Pos;
        }
        if (stackLevelIntake == 3) {
//            FBLeft.setPosition(FBLeftIntakeStack3Pos);
//            FBRight.setPosition(FBRightIntakeStack3Pos);
            leftPos = FBLeftIntakeStack3Pos;
            rightPos = FBRightIntakeStack3Pos;
        }
        if (stackLevelIntake == 2) {
//            FBLeft.setPosition(FBLeftIntakeStack2Pos);
//            FBRight.setPosition(FBRightIntakeStack2Pos);
            leftPos = FBLeftIntakeStack2Pos;
            rightPos = FBRightIntakeStack2Pos;
        }
        if (stackLevelIntake == 0) {
//            FBLeft.setPosition(FBLeftIntakeStack2Pos);
//            FBRight.setPosition(FBRightIntakeStack2Pos);
            leftPos = FBLeftIntakeHigh;
            rightPos = FBRightIntakeHigh;
        }

        if (extensionRange >= 1) {
            leftPos -= 0.01;
            rightPos += 0.01;
        }
        if (extensionRange >= 2) {
            leftPos -= 0.01;
            rightPos += 0.01;
        }
        if (extensionRange >= 3) {
            leftPos -= 0.01;
            rightPos += 0.01;
        }

        FBLeft.setPosition(leftPos);
        FBRight.setPosition(rightPos);
    }

    public void FBToIntakeTele() {
        double leftPos = FBLeftIntakePos;
        double rightPos = FBRightIntakePos;

//        if (stackLevelIntake == 5) {
////            FBLeft.setPosition(FBLeftIntakeStack5Pos);
////            FBRight.setPosition(FBRightIntakeStack5Pos);
//            leftPos = FBLeftIntakeStack5PosTele;
//            rightPos = FBRightIntakeStack5PosTele;
//        }
//        if (stackLevelIntake == 4) {
////            FBLeft.setPosition(FBLeftIntakeStack4Pos);
////            FBRight.setPosition(FBRightIntakeStack4Pos);
//            leftPos = FBLeftIntakeStack4PosTele;
//            rightPos = FBRightIntakeStack4PosTele;
//        }
//        if (stackLevelIntake == 3) {
////            FBLeft.setPosition(FBLeftIntakeStack3Pos);
////            FBRight.setPosition(FBRightIntakeStack3Pos);
//            leftPos = FBLeftIntakeStack3PosTele;
//            rightPos = FBRightIntakeStack3PosTele;
//        }
//        if (stackLevelIntake == 2) {
////            FBLeft.setPosition(FBLeftIntakeStack2Pos);
////            FBRight.setPosition(FBRightIntakeStack2Pos);
//            leftPos = FBLeftIntakeStack2PosTele;
//            rightPos = FBRightIntakeStack2PosTele;
//        }
//        if (stackLevelIntake == 0) {
////            FBLeft.setPosition(FBLeftIntakeStack2Pos);
////            FBRight.setPosition(FBRightIntakeStack2Pos);
//            leftPos = FBLeftIntakeHigh;
//            rightPos = FBRightIntakeHigh;
//        }

        FBLeft.setPosition(leftPos);
        FBRight.setPosition(rightPos);
    }

    public void FBToIntake(int stackLevelIntake) {
        double leftPos = FBLeftIntakePos;
        double rightPos = FBRightIntakePos;

        if (stackLevelIntake == 5) {
//            FBLeft.setPosition(FBLeftIntakeStack5Pos);
//            FBRight.setPosition(FBRightIntakeStack5Pos);
            leftPos = FBLeftIntakeStack5Pos;
            rightPos = FBRightIntakeStack5Pos;
        }
        if (stackLevelIntake == 4) {
//            FBLeft.setPosition(FBLeftIntakeStack4Pos);
//            FBRight.setPosition(FBRightIntakeStack4Pos);
            leftPos = FBLeftIntakeStack4Pos;
            rightPos = FBRightIntakeStack4Pos;
        }
        if (stackLevelIntake == 3) {
//            FBLeft.setPosition(FBLeftIntakeStack3Pos);
//            FBRight.setPosition(FBRightIntakeStack3Pos);
            leftPos = FBLeftIntakeStack3Pos;
            rightPos = FBRightIntakeStack3Pos;
        }
        if (stackLevelIntake == 2) {
//            FBLeft.setPosition(FBLeftIntakeStack2Pos);
//            FBRight.setPosition(FBRightIntakeStack2Pos);
            leftPos = FBLeftIntakeStack2Pos;
            rightPos = FBRightIntakeStack2Pos;
        }
        if (stackLevelIntake == 0) {
//            FBLeft.setPosition(FBLeftIntakeStack2Pos);
//            FBRight.setPosition(FBRightIntakeStack2Pos);
            leftPos = FBLeftIntakeHigh;
            rightPos = FBRightIntakeHigh;
        }

        FBLeft.setPosition(leftPos);
        FBRight.setPosition(rightPos);
    }

    public boolean isFBArmPosToIntake() {
        return FBLeft.getPosition() > 0.4;
    }

    public void FBToTransferHigh() {
        FBLeft.setPosition(FBLeftTransferHighPos);
        FBRight.setPosition(FBRightTransferHighPos);
    }

    public void FBToTransferLow() {
        FBLeft.setPosition(FBLeftTransferLowPos);
        FBRight.setPosition(FBRightTransferLowPos);
    }

    public void FBToTransfer() {
        FBLeft.setPosition(FBLeftTransferPos);
        FBRight.setPosition(FBRightTransferPos);
    }

    public void FBToGround() {
        FBLeft.setPosition(FBLeftGroundJunctionPos);
        FBRight.setPosition(FBRightGroundJunctionPos);
    }

    public void FBToGroundSlow() throws InterruptedException {
//        for (double i = 0.4; i > 0; i -= 0.4) {
//            FBLeft.setPosition(FBLeftGroundJunctionPos - i);
//            FBRight.setPosition(FBRightGroundJunctionPos + i);
//            sleep(500);
//        }
        FBLeft.setPosition(FBLeftGroundJunctionHighPos);
        FBRight.setPosition(FBRightGroundJunctionHighPos);
    }

    public void FBToRest() {
        FBLeft.setPosition(FBLeftRestPos);
        FBRight.setPosition(FBRightRestPos);
    }

    public void FBToLowJunction() {
        FBLeft.setPosition(FBLeftLowJunctionPos);
        FBRight.setPosition(FBRightLowJunctionPos);
    }

    public void FBToGroundRaised() {
        FBLeft.setPosition(FBLeftGroundRaisedPos);
        FBRight.setPosition(FBRightGroundRaisedPos);
    }

    public void FBToAfterInitPos() {
        FBLeft.setPosition(FBLeftAfterInitPos);
        FBRight.setPosition(FBRightAfterInitPos);
    }

    public void openClaw() {
        claw.setPosition(openClawPos);
    }

    public void openClawPartially() {
        claw.setPosition(partiallyOpenClawPos);
    }

    public void openClawAuto() {
        claw.setPosition(openClawAutoPos);
    }

    public void closeClaw() {
        claw.setPosition(closeClawTelePos);
    }

    public void closeClawLowJunction() {
        claw.setPosition(closeClawAutoPos);
    }

    public void rotateClawToTransfer() {
        clawRotate.setPosition(clawRotateToTransferPos);
    }

    public void rotateClawToIntakeLeftAuto() {
        clawRotate.setPosition(clawRotateToIntakePosLeftAuto);
    }

    public void rotateClawToIntakeRightAuto() {
        clawRotate.setPosition(clawRotateToIntakePosRightAuto);
    }

    public void rotateClawToIntakeTeleOp() {
        clawRotate.setPosition(clawRotateToIntakePosTeleOp);
    }

    public void engageConeStabilizer() { coneStabilizer.setPosition(coneStabilizerEngagedPos); }

    public void disengageConeStabilizer() { coneStabilizer.setPosition(coneStabilizerDisengagedPos); }

    public void retractCollectorSlidesTele() throws InterruptedException {
        collectorTop.setPower(-0.95);
        collectorBottom.setPower(-0.95);

        while (collectorTop.getCurrentPosition() > 20) {}
        sleep(200);

        collectorTop.setPower(0.0);
        collectorBottom.setPower(0.0);
    }

    public void retractCollectorSlides(String mode) throws InterruptedException {
        collectorRetractionController = new PIDController(0.006, 0.0, 0.0001, maxCollectorRetractionPower);

        if (mode.equals("TELE")) {
            rotateClawToTransfer();
        }

        while (collectorTop.getCurrentPosition() > maxAllowableCollectorError) {
            double targetPower = collectorRetractionController.calculateUsingEncoder(0, collectorTop.getCurrentPosition()); // + collectorSlidesSlip
            targetPower -= 0.2;
            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);

            if (mode.equals("AUTO") && collectorTop.getCurrentPosition() < collectorSlidesStack5 - 100) {
                rotateClawToTransfer();
            }
        }

        collectorTop.setPower(0);
        collectorBottom.setPower(0);

        collectorEncOffset = collectorTop.getCurrentPosition();
    }

    // TODO: add pid method
    public void extendCollectorSlides(double targetPos) {
        collectorTop.setPower(0.65);
        collectorBottom.setPower(0.65);

        while (Math.abs(collectorTop.getCurrentPosition()) < targetPos) {}

        collectorTop.setPower(0);
        collectorBottom.setPower(0);
    }

    public void setCollectorSlidesPower(double pow) {
        collectorTop.setPower(pow);
        collectorBottom.setPower(pow);
    }

    public void setDepositSlidesPower(double pow) {
        dropperBack.setPower(pow);
        dropperFront.setPower(pow);
    }

    public double retrieveDropperEncoderCount() { return dropperBack.getCurrentPosition(); }

    public double retrieveCollectorEncoderCount() { return collectorTop.getCurrentPosition(); }

    public void collectCone() throws InterruptedException {
        closeClawTele();
        sleep(200);
        disengageConeStabilizer();
        sleep(200);
        FBToTransferHigh();
        sleep(175); // 600
        rotateClawToTransfer();

        retractCollectorSlidesTele();
//        sleep(300);

        FBToTransferLow();
        sleep(1000);
        openClawPartially();
        sleep(200); // 550
        FBToRest();
        sleep(200); // 150
        rotateClawToIntakeTeleOp();
        sleep(200);
    }

    public void transferConeCycles() throws InterruptedException {
//        closeClaw();
//        sleep(150);
//        FBToTransfer();
//        rotateClawToTransfer();
//
//        retractCollectorSlides("TELE");
//
//        sleep(200);
//        openClawAuto();
//        sleep(500);
//        FBToIntake(1);

        ElapsedTime timer = new ElapsedTime();
        long initialTime = timer.time(TimeUnit.MILLISECONDS);
        while (getCollectorSensorReading() - collectableConeDistance > maxAllowableCollectorError) {
            if (timer.time(TimeUnit.MILLISECONDS) - initialTime > distanceSensorTimeout) {
                break;
            }
            double targetPower = collectorExtensionController.calculateUsingDistanceSensor(getCollectorSensorReading() - collectableConeDistance); // + collectorSlidesSlip
            collectorTop.setPower(targetPower + 0.2);
            collectorBottom.setPower(targetPower + 0.2);
        }

        closeClaw();
        sleep(200); // 600
        FBToTransferHigh();
        sleep(300);
        rotateClawToTransfer();

        retractCollectorSlides("TELE");
        sleep(300);

        FBToTransferLow();
        sleep(400);
        openClaw();
        sleep(150); // 300
        FBToIntakeTele(1, 1);
        sleep(150); // 150
        rotateClawToIntakeTeleOp();
        sleep(100);
    }

    public void collectConeCycles() throws InterruptedException {
        FBToIntake(1);
        openClaw();
        sleep(100);

        // Extend slides
        collectorTop.setPower(1.0);
        collectorBottom.setPower(1.0);

        rotateClawToIntakeTeleOp();

//        while (Math.abs(collectorTop.getCurrentPosition()) < 400) {}
//        while (collectorTop.getCurrentPosition() < collectConesCyclesPos - 60) {
////            double targetPower = collectorExtensionController.calculateUsingEncoder(collectConesCyclesPos - 60, collectorTop.getCurrentPosition()); // + collectorSlidesSlip
////            collectorTop.setPower(targetPower);
////            collectorBottom.setPower(targetPower);
////        }
////        while (collectConesCyclesPos - collectorTop.getCurrentPosition() < maxAllowableCollectorError) {
////            collectorTop.setPower(0.3);
////            collectorBottom.setPower(0.3);
////        }

        while (collectConesCyclesPos - collectorTop.getCurrentPosition() > maxAllowableCollectorError) {
            double targetPower = collectorExtensionController.calculateUsingEncoder(collectConesCyclesPos, collectorTop.getCurrentPosition()); // + collectorSlidesSlip
            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);
        }

        collectorTop.setPower(0);
        collectorBottom.setPower(0);
    }

    public void extendDropperAutoHightele(int mode, Telemetry telemetry, Thread autoCollectCone) throws InterruptedException {

        boolean collectedConeProcessStarted = false;
        boolean transferConeProcessStarted = false;

        if (mode == 3) {
//            collectCone();

            dropperFront.setPower(0.8); //0.8
            dropperBack.setPower(0.8);

            while(dropperBack.getCurrentPosition() < (dropperSlidesHighPosAuto - 60)) {
                if (dropperBack.getCurrentPosition() >= (dropperSlidesHighPosAuto*0.07) && !collectedConeProcessStarted) {
                    autoCollectCone.start();
                    collectedConeProcessStarted = true;
                }
                telemetry.addData("Dropper Pos", dropperBack.getCurrentPosition());
                telemetry.update();
            }
            dropperFront.setPower(0.1);
            dropperBack.setPower(0.1);
            telemetry.addData("After Extended Pos", dropperBack.getCurrentPosition());
            telemetry.update();
        }
        else if (mode == 1) {
            dropperFront.setPower(-0.7);
            dropperBack.setPower(-0.7);
            while (dropperBack.getCurrentPosition() > 20) {
                if (dropperBack.getCurrentPosition() <= (dropperSlidesHighPosAuto*0.98) && !transferConeProcessStarted) {
                }
            }
            dropperFront.setPower(0);
            dropperBack.setPower(0);
            telemetry.addData("After Retracted Pos", dropperBack.getCurrentPosition());
            telemetry.update();
        }
    }

    public void lowerDepositSlidesCycles() {
        dropperFront.setPower(-1.0);
        dropperBack.setPower(-1.0);

//        double startPos = dropperBack.getCurrentPosition();
//        transferCone.start();

        while(dropperBack.getCurrentPosition() > (20)) {
//            if (dropperBack.getCurrentPosition() < startPos && !transferCone.isAlive()) {
//
//            }
        }

        dropperFront.setPower(0);
        dropperBack.setPower(0);
    }

    public void depositCone(Telemetry telemetry, String level) throws InterruptedException {
        if (level.equals("high") || level.equals("mid")) {
//            rotateClawToIntake();
            raiseDepositSlides(telemetry, level);
            lowerDepositSlides(telemetry);
        }
        else if (level.equals("low") || level.equals("ground")) {
            openClaw();
        }
    }

    // AUTO FUNCTIONS
    public void brakeMotorPowers() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void raiseDepositSlides(Telemetry telemetry, String level) throws InterruptedException {
//        dropperSlidesSlip = dropperBack.getCurrentPosition();
        double pos = dropperSlidesHighPosAuto; //  + dropperSlidesSlip
        if (level.equals("mid")) {
            pos = dropperSlidesMidPos;
        }
        boolean stabilizerEngaged = false;
        depositExtensionController = new PIDController(0.022, 0.0, 0.0002, maxDepositExtensionPower);

//        if (level.equals("mid")) {
//            pos = dropperSlidesMidPos;
//        }

        while (pos - dropperBack.getCurrentPosition() > maxAllowableDepositError) {
            if (dropperBack.getCurrentPosition() > 100 && !stabilizerEngaged) {
                engageConeStabilizer();
                stabilizerEngaged = true;
            }
            double targetPower = depositExtensionController.calculateUsingEncoder(pos, dropperBack.getCurrentPosition());
            dropperBack.setPower(targetPower);
            dropperFront.setPower(targetPower);

            telemetry.addData("Relative Dropper Pos", dropperBack.getCurrentPosition()); //  - dropperSlidesSlip
            telemetry.addData("Target power", targetPower);
            telemetry.update();
        }

        dropperFront.setPower(0.1);
        dropperBack.setPower(0.1);
        sleep(200);
        disengageConeStabilizer();
    }

    public void lowerDepositSlides(Telemetry telemetry) throws InterruptedException {
        depositRetractionController = new PIDController(0.022, 0.0, 0.0002, maxDepositRetractionPower);
        ElapsedTime timer = new ElapsedTime();
        long initialTime = timer.time(TimeUnit.MILLISECONDS);
        while (dropperBack.getCurrentPosition() - dropperSlidesRetractedPos > maxAllowableDepositError) {
            if (timer.time(TimeUnit.MILLISECONDS) - initialTime > distanceSensorTimeout) {
                break;
            }
            double targetPower = depositRetractionController.calculateUsingEncoder(dropperSlidesRetractedPos, dropperBack.getCurrentPosition());
            dropperBack.setPower(targetPower);
            dropperFront.setPower(targetPower);

            telemetry.addData("Relative Dropper Pos", dropperBack.getCurrentPosition()); //  - dropperSlidesSlip
            telemetry.update();
            if (timer.seconds() > 3) {
                telemetry.addData("Broke out of loop", dropperBack.getCurrentPosition());
                telemetry.update();
                break;
            }
        }

        dropperFront.setPower(0.1);
        dropperBack.setPower(0.1);

//        sleep(200);
//        double currentPosition = dropperBack.getCurrentPosition();
//        while (dropperBack.getCurrentPosition() - dropperSlidesInitPos > maxAllowableDepositError) {
//            if (currentPosition - dropperBack.getCurrentPosition() > 100) {
//                disengageConeStabilizer();
//            }
//            double targetPower = depositController.calculate(dropperSlidesInitPos, dropperBack.getCurrentPosition());
//            dropperBack.setPower(targetPower);
//            dropperFront.setPower(targetPower);
//
//            telemetry.addData("Relative Dropper Pos", dropperBack.getCurrentPosition()); //  - dropperSlidesSlip
//            telemetry.update();
//        }
//
//        dropperFront.setPower(0.1);
//        dropperBack.setPower(0.1);
    }

    public void extendCollectCone(Telemetry telemetry, int stackLevel, String mode) throws InterruptedException {
        double collectorSlidesStack = collectorSlidesStack5;
        collectorExtensionController = new PIDController(0.006, 0.0, 0.0001, maxCollectorExtensionPower);
        if (stackLevel == 5) {
            FBLeft.setPosition(FBLeftIntakeStack5Pos);
            FBRight.setPosition(FBRightIntakeStack5Pos);
            collectorSlidesStack = collectorSlidesStack5;
        }
        if (stackLevel == 4) {
            FBLeft.setPosition(FBLeftIntakeStack4Pos);
            FBRight.setPosition(FBRightIntakeStack4Pos);
            collectorSlidesStack = collectorSlidesStack4;
        }
        if (stackLevel == 3) {
            FBLeft.setPosition(FBLeftIntakeStack3Pos);
            FBRight.setPosition(FBRightIntakeStack3Pos);
            collectorSlidesStack = collectorSlidesStack3;
        }
        if (stackLevel == 2) {
            FBLeft.setPosition(FBLeftIntakeStack2Pos);
            FBRight.setPosition(FBRightIntakeStack2Pos);
            collectorSlidesStack = collectorSlidesStack2;
        }
        if (stackLevel == 1) {
            FBLeft.setPosition(FBLeftIntakeStack1Pos);
            FBRight.setPosition(FBRightIntakeStack1Pos);
            collectorSlidesStack = collectorSlidesStack1;
        }

//        collectorSlidesSlip = collectorTop.getCurrentPosition();
        openClaw();
        if (mode.equals("right")) {
            rotateClawToIntakeRightAuto();
        }
        else if (mode.equals("left")) {
            rotateClawToIntakeLeftAuto();
        }
        else {
            rotateClawToIntakeTeleOp();
        }
//        sleep(1000);

        // Extend slides

        // collectorSlidesStack + collectorSlidesSlip - collectorTop.getCurrentPosition() > maxAllowableCollectorError
        // getCollectorSensorReading() > collectableConeDistance
        // collectorSlidesStack + collectorSlidesSlip - collectorTop.getCurrentPosition() > 5
        // getCollectorSensorReading() - collectableConeDistance > maxAllowableCollectorError
        ElapsedTime timer = new ElapsedTime();
        long initialTime = timer.time(TimeUnit.MILLISECONDS);

        while (collectorTop.getCurrentPosition() - extensionPidThreshold > maxAllowableCollectorError) {
            double targetPower = collectorExtensionController.calculateUsingEncoder(extensionPidThreshold, collectorTop.getCurrentPosition()); // + collectorSlidesSlip
            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);
        }
        while (getCollectorSensorReading() > collectableConeDistance) {
            if (timer.time(TimeUnit.MILLISECONDS) - initialTime > distanceSensorTimeout) {
                break;
            }
//            double targetPower = collectorExtensionController.calculateUsingDistanceSensor(getCollectorSensorReading() - collectableConeDistance); // + collectorSlidesSlip
            double targetPower = collectorExtensionController.calculateUsingDistanceSensor(getCollectorSensorReading());
            collectorTop.setPower(targetPower);
            collectorBottom.setPower(targetPower);

            telemetry.addData("CD Reading ", getCollectorSensorReading());
            telemetry.addData("Collector Pos ".concat(Integer.toString(stackLevel)), collectorTop.getCurrentPosition());
            telemetry.addData("Collector Pos Slip".concat(Integer.toString(stackLevel)), collectorSlidesSlip);
            telemetry.addData("Error with Slip".concat(Integer.toString(stackLevel)), collectorSlidesStack + collectorSlidesSlip - collectorTop.getCurrentPosition());
            telemetry.update();

//            if (getCollectorSensorReading() < collectableConeDistance) {
//                break;
//            }
        }

        collectorTop.setPower(0.05);
        collectorBottom.setPower(0.05);

//        sleep(300);

//        collectorSlidesSlip = 0;
        telemetry.update();
    }

    public boolean retractCollectConeSafe(Telemetry telemetry, int currentStackLevel, String mode) throws InterruptedException {
        closeClawAuto();
        sleep(200); // 600
        FBToTransferHigh();
        sleep(300);

        retractCollectorSlides("AUTO");
        sleep(300);

        FBToTransferLow();
        sleep(400);
        openClaw();
        sleep(300); // 550
        if (getCollectorSensorReading() > collectableConeDistance + 10) {
            FBToRest();
            rotateClawToIntakeTeleOp();
            FBToIntake(1);
            return false;
        }
        FBToIntake(currentStackLevel - 1);
        sleep(150); // 150
        if (mode.equals("right")) {
            rotateClawToIntakeRightAuto();
        }
        else if (mode.equals("left")) {
            rotateClawToIntakeLeftAuto();
        }
        else {
            rotateClawToIntakeTeleOp();
        }
        sleep(100);
        return true;
    }

    public void retractCollectCone(Telemetry telemetry, int currentStackLevel, String mode) throws InterruptedException {
        closeClawAuto();
        sleep(300); // 600
        FBToTransferHigh();
        sleep(350);
//        while (Math.abs(FBRight.getPosition() - FBRightTransferHighPos) > 0.1) {}
        rotateClawToTransfer();

        retractCollectorSlides("AUTO");
        sleep(200);

        FBToTransferLow();
        sleep(300);
        openClaw();
        sleep(200); // 550
        FBToIntake(currentStackLevel - 1);
        sleep(100); // 150
        if (mode.equals("right")) {
            rotateClawToIntakeRightAuto();
        }
        else if (mode.equals("left")) {
            rotateClawToIntakeLeftAuto();
        }
        else {
            rotateClawToIntakeTeleOp();
        }
//        sleep(100);
    }

    public double getCollectorSensorReading() {
        return collectorSensor.getDistance(DistanceUnit.MM);
    }

    public void closeClawAuto() {
        claw.setPosition(closeClawAutoPos);
    }

    public void closeClawTele() {
        claw.setPosition(closeClawTelePos);
    }

//    public double getDepositSensorDistanceReading() {
//        return depositSensorDistance.getDistance(DistanceUnit.MM);
//    }

//    public double getDepositSensorColorReading() {
//        return depositSensorColor.argb();
//    }

    public void retrieve() throws InterruptedException {
        closeClaw();
        sleep(250);

        FBToRest();
        sleep(300);
        rotateClawToTransfer();
        sleep(350);
    }

    public void transfer() throws InterruptedException {
        FBToTransfer();
        sleep(400);
        openClawAuto();
        sleep(350);
        FBToRest();
        sleep(100);
        openClaw();
    }

}
