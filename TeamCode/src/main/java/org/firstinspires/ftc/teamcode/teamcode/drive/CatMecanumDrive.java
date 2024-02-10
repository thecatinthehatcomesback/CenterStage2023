package org.firstinspires.ftc.teamcode.teamcode.drive;

import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.teamcode.drive.DriveConstants.encoderTicksToInches;

import android.util.Log;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class CatMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(8, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.03448;
    public static double kV = DriveConstants.kV;
    public static double kA = DriveConstants.kA;
    public static double kStatic = DriveConstants.kStatic;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;
    public double leftInches = 0;
    public double rightInches = 0;
    private double lastTime = 0;
    private double lastLeftError = 0;
    private double lastRightError = 0;
    private double rightIntegralSum = 0;
    private double leftIntegralSum = 0;
    private ElapsedTime pidTimer = null;
    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public CatMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(.5);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

        pidTimer = new ElapsedTime();
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
        return 0;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return 0.0;
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
    /**
     * Will scale down our calculated power numbers if they are greater than 1.0.  If the values
     * were greater than 1.0, the motors would spin at their max powers.  This would limit precise
     * paths the robot could take, thus we created this method to "scale down" all the values by
     * creating a scale factor so that there is a proportional difference in all the motor powers,
     * giving the robot better mobility, especially with mecanum wheels.
     *
     * @param leftFrontValue  Prospective value for motor power that may be scaled down.
     * @param rightFrontValue Prospective value for motor power that may be scaled down.
     * @param leftBackValue   Prospective value for motor power that may be scaled down.
     * @param rightBackValue  Prospective value for motor power that may be scaled down.
     * @return what should be multiplied with all the other motor powers to get a good proportion.
     */
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /*
        PLANS:
        1: Look at all motor values
        2: Find the highest absolute value (the "scalor")
        3: If the highest value is not more than 1.0, we don't need to change the values
        4: But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
        5: Finally, we pass OUT the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value:
        for (int i = 0; i + 1 < values.length; i++) {
            if (values[i] > scalor) {
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done...
        if (scalor > 1.0) {
            // Get the reciprocal:
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don't have to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }

    public void updateDistance(){
        double newLeftInches = leftDistance.getDistance(DistanceUnit.INCH);
        double newRightInches = rightDistance.getDistance(DistanceUnit.INCH);
        leftInches = leftInches * 0.66 + newLeftInches * .34;
        rightInches = rightInches * 0.66 + newRightInches * .34;
    }
    public boolean scoreHexTeleop(){
        double curTime = pidTimer.seconds();

        if(curTime - lastTime > .5){
            lastTime = curTime;
            lastLeftError = 0;
            leftIntegralSum = 0;
            rightIntegralSum = 0;
        }
        double Kp = 0.05;
        double Ki = 0.035;
        double Kd = 0.015; // was 0.0015


        double goalDistance = 4.2;
        double leftError = goalDistance  - leftInches;
        double leftDerivative = (leftError - lastLeftError)/(curTime - lastTime);
        leftIntegralSum += leftError * (curTime-lastTime);
        leftFront.setPower(Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative);
        leftRear.setPower(Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative);

        double rightError = goalDistance  - rightInches;
        double rightDerivative = (rightError - lastRightError)/(curTime - lastTime);
        rightIntegralSum += rightError * (curTime-lastTime);

        rightFront.setPower(Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative);
        rightRear.setPower(Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative);

        lastTime = curTime;
        lastLeftError = leftError;
        lastRightError = rightError;
        Log.d("catbot",String.format("Lpow: %.3f Lerror: %.1f Lder: %.2f Li: %.2f Rpow: %.3f Rerror: %.1f Rder: %.2f Ri: %.2f",
                    Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative,leftError,leftDerivative,leftIntegralSum,
                Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative,rightError,rightDerivative,rightIntegralSum));
        if((leftError < 0.2 && leftError >= 0 && rightError < 0.2 && leftError >= 0)
                || leftError < -300 || rightError < -300){
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);
            leftRear.setPower(0);
            return false;
        }
        return true;
    }
    public void scoreHexAutonomous(){
        double curTime = pidTimer.seconds();

        if(curTime - lastTime > .5){
            lastTime = curTime;
            lastLeftError = 0;
            leftIntegralSum = 0;
            rightIntegralSum = 0;
        }
        double Kp = 0.05;
        double Ki = 0.035;
        double Kd = 0.015; // was 0.0015


        double goalDistance = 3.8;
        double leftError = goalDistance  - leftInches;
        double leftDerivative = (leftError - lastLeftError)/(curTime - lastTime);
        leftIntegralSum += leftError * (curTime-lastTime);
        leftFront.setPower(Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative);
        leftRear.setPower(Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative);

        double rightError = goalDistance  - rightInches;
        double rightDerivative = (rightError - lastRightError)/(curTime - lastTime);
        rightIntegralSum += rightError * (curTime-lastTime);

        rightFront.setPower(Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative);
        rightRear.setPower(Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative);

        lastTime = curTime;
        lastLeftError = leftError;
        lastRightError = rightError;
        Log.d("catbot",String.format("Lpow: %.3f Lerror: %.1f Lder: %.2f Li: %.2f Rpow: %.3f Rerror: %.1f Rder: %.2f Ri: %.2f",
                Kp * leftError + Ki * leftIntegralSum + Kd * leftDerivative,leftError,leftDerivative,leftIntegralSum,
                Kp * rightError + Ki * rightIntegralSum + Kd * rightDerivative,rightError,rightDerivative,rightIntegralSum));
        if((leftError < 0.2 && leftError >= 0 && rightError < 0.2 && leftError >= 0)
                || leftError < -300 || rightError < -300){
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightRear.setPower(0);
            leftRear.setPower(0);
        }
    }
}
