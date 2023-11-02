package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

/**
 * CatHW_DriveOdo.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the drive train using odometry modules as position givers.  This file is used by the
 * new autonomous OpModes to run multiple operations at once with odometry.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * NOTE: All names are lower case and have underscores between words.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_DriveOdo extends CatHW_Subsystem
{
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    //----------------------------------------------------------------------------------------------
    // Odometry Module Constants:                               TODO: Are these constants updated???
    //----------------------------------------------------------------------------------------------

    /**
     * The number of encoder ticks per one revolution of the odometry wheel.
     * 8192 ticks for a REV encoder from REV Robotics.
     */
    private static final double ODO_COUNTS_PER_REVOLUTION = 8192;
    /** The measurement of the odometry wheel diameter for use in calculating circumference. */
    private static final double ODO_WHEEL_DIAMETER_INCHES = 2.0;
    /**
     * The amount of odometry encoder ticks equal to movement of 1 inch.  Used for conversion in the
     * robot's positioning algorithms so that when a user inputs (X,Y) coordinates in inches, those
     * can be converted into encoder ticks.
     */
    static final double ODO_COUNTS_PER_INCH     = ODO_COUNTS_PER_REVOLUTION /
            (ODO_WHEEL_DIAMETER_INCHES * Math.PI);


    //TODO: Other attributes/field should get some Javadoc sometime...
    private double targetX;
    private double targetY;
    private double targetTheta;
    private double xMin;
    private double xMax;
    private double yMin;
    private double yMax;
    private double thetaMin;
    private double thetaMax;
    private double maxPower;
    private double strafePower;
    private  double prevzVal;
    private double prevSec;

    private double prevX;
    private double prevY;
    private double prevTheta;

    /** ArrayList of Points that the robot will drive towards. */
    private ArrayList<CatType_CurvePoint> targetPoints;

    /** A default follow radius for our Pure Pursuit Algorithms. */
    private final double DEFAULT_FOLLOW_RADIUS = 20.0;
    /** The following distance between the robot and any point on the line paths. */
    private double followRadius = DEFAULT_FOLLOW_RADIUS;

    private ElapsedTime movementTimer = new ElapsedTime();
    // Turn stuff:
    int targetAngleZ;
    int baseDelta;
    boolean clockwiseTurn;

    private boolean isNonStop;

    private double tolerance = 0.5; //inches

    double timeout = 0;

    static boolean isDone;


    ElapsedTime runTime = new ElapsedTime();

    // Motors:
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    public AnalogInput distanceSensor = null;



    /** Enumerated type for the style of drive the robot will make. */
    private enum DRIVE_METHOD {
        TRANSLATE,
        TURN,
        INTAKE,
        PURE_PURSUIT
    }
    /** Variable to keep track of the DRIVE_METHOD that's the current style of robot's driving. */
    private DRIVE_METHOD currentMethod;



    //----------------------------------------------------------------------------------------------
    // Public OpMode Members
    //----------------------------------------------------------------------------------------------


    /* Constructor */
    public CatHW_DriveOdo(CatHW_Async mainHardware){
        super(mainHardware);

    }

    /**
     * Initialize standard Hardware interfaces in the CatHW_DriveOdo hardware.
     *
     * @throws InterruptedException in case of error.
     */
    public void init()  throws InterruptedException  {


        // Define and Initialize Motors: //
        leftFrontMotor = hwMap.dcMotor.get("leftFront");
        rightFrontMotor = hwMap.dcMotor.get("rightFront");
        leftRearMotor = hwMap.dcMotor.get("leftRear");
        rightRearMotor = hwMap.dcMotor.get("rightRear");
        //distanceSensor = hwMap.analogInput.get("distance");


        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);


        // Define motor zero power behavior: //
        setDriveToBrake();

        // Set motor modes: //
        resetDriveEncoders();
        setDriveRunWithoutEncoders();

        // Set all motors to run at no power so that the robot doesn't move during init: //
        setDrivePowers(0, 0, 0, 0);

        // Sets enums to a default value: //
        currentMethod = DRIVE_METHOD.TRANSLATE;
        timeout = 0;
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

    }

    public void setNormalTolerance(){
        tolerance = 0.5;
    }
    public void setLooseTolerance(){
        tolerance= 2;
    }
    public void setTightTolerance(){
        tolerance = 0.1;
    }

    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------



    /**
     * Calls the translateDrive() method and adds in a waitUntilDone() afterwards so that the
     * autonomous code is cleaner without having so many waitUntilDone() methods clogging up the
     * view.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void quickDrive(double x, double y, double theta, double power, double timeoutS){

        translateDrive(x,y,power,theta,timeoutS);
        waitUntilDone();
    }

    public void quickIntakeDrive(double power, double timeoutS){
        intakeDrive(power,timeoutS);

        waitUntilDone();
    }


    /**
     * Used to move the robot across the field.  The robot can also TURN while moving along the path
     * in order for it to face a certain by the end of its path.  This method assumes the robot has
     * odometry modules which give an absolute position of the robot on the field.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void translateDrive(double x, double y, double power, double theta, double timeoutS) {

        currentMethod = DRIVE_METHOD.TRANSLATE;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        targetTheta = theta;
        Pose2d poseEstimate = drive.getPoseEstimate();


        if (isNonStop){
            //if the last drive was nonstop
            isNonStop = false;

            Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                    .splineTo(new Vector2d(x, y), Math.toRadians(theta))
                    .build();
            drive.followTrajectory(traj1);
        }else {
            //if the last drive was normal
            Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                    .splineTo(new Vector2d(x, y), Math.toRadians(theta))
                    .build();
            drive.followTrajectoryAsync(traj1);

        }
        drive.setWeightedDrivePower(new Pose2d(
                power,
                power,
                power
        ));


        // Reset timer once called
        runTime.reset();
        movementTimer.reset();
        prevX = 0;
        prevY = 0;
        prevTheta = 0;
    }

    /**
     * Nonstop TRANSLATE.  TODO: Add Javadoc here.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param finishedXMin
     * @param finishedXMax
     * @param finishedYMin
     * @param finishedYMax
     * @param finishedThetaMin
     * @param finishedThetaMax
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void translateDrive(double x, double y, double power, double theta, double finishedXMin,
                               double finishedXMax, double finishedYMin, double finishedYMax,
                               double finishedThetaMin, double finishedThetaMax, double timeoutS){

        currentMethod = DRIVE_METHOD.TRANSLATE;
        timeout = timeoutS;
        isDone = false;
        targetX = x;
        targetY = y;
        strafePower = power;
        targetTheta = theta;
        xMin = finishedXMin;
        xMax = finishedXMax;
        yMin = finishedYMin;
        yMax = finishedYMax;
        thetaMin = finishedThetaMin;
        thetaMax = finishedThetaMax;
        maxPower = power;

        // Power update Thread:
        if (isNonStop){
            //if the last drive was nonstop
            // motionProfile.setNonStopTarget(x, y, power, realSense.getXPos(), realSense.getYPos());
        }else {
            //if the last drive was normal
            // motionProfile.setTarget(x, y, power, realSense.getXPos(), realSense.getYPos());

        }

        //set it so the next one will be nonstop
        isNonStop = true;

        // Reset timer once called
        runTime.reset();

    }
    /**
     * Overloaded method (fancy way of saying same method header with different parameter list) that calls the other
     * pursuitDrive() method, but automatically sets the followRadius to the DEFAULT_FOLLOW_RADIUS constant.
     *
     * @param points is an ArrayList of Points that make up the user-defined path the robot will follow.
     * @param power at which the robot's max speed will be set to using motion profiling.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(ArrayList<CatType_CurvePoint> points, double power, double timeoutS) {

        pursuitDrive(points, power, DEFAULT_FOLLOW_RADIUS, timeoutS);
    }

    /**
     * Used to move the robot across the field.  The robot can also TURN while moving along the path in order for it to
     * face a certain by the end of its path.  This method assumes the robot has odometry modules which give an absolute
     * position of the robot on the field.
     *
     * @param points is an ArrayList of Points that make up the user-defined path the robot will follow.
     * @param power at which the robot's max speed will be set to using motion profiling.
     * @param followRadius is the distance between the robot and the point ahead of it on the path that it will
     *         choose to follow.
     * @param timeout is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(ArrayList<CatType_CurvePoint> points, double power, double followRadius, double timeout) {

        currentMethod = DRIVE_METHOD.PURE_PURSUIT;
        this.timeout = timeout;
        isDone = false;
        targetPoints = points;

        // targetPoints.add(0, new CatType_CurvePoint(realSense.getXPos(), realSense.getYPos(),realSense.getRotation()));
        this.followRadius = followRadius;
        for(int i = 0; i<targetPoints.size(); i++){
            Log.d("catbot",String.format("Pursuit Point %.2f %.2f %.2f",targetPoints.get(i).x,targetPoints.get(i).y, targetPoints.get(i).theta ));

        }

        //CatType_CurvePoint targetPoint = updatesThread.powerUpdate.getFollowPoint(targetPoints,
        //        updatesThread.positionUpdate.returnRobotPointInches(), followRadius);


        // Power update Thread:
        if (isNonStop) {
            // If the last drive method call was nonstop:
            //motionProfile.setNonStopTarget(points, power, followRadius);
        } else {
            // If the last drive method call was normal:
            //motionProfile.setTarget(points, power, followRadius);
        }

        // Set it so the next one will be nonstop.
        isNonStop = false;

        // Reset timer once called.
        runTime.reset();
        movementTimer.reset();
    }

    /**
     * Nonstop TRANSLATE.
     *
     * @param x is the new X coordinate the robot drives to.
     * @param y is the new Y coordinate the robot drives to.
     * @param power at which robot max speed can be set to using motion profiling.
     * @param theta is the angle at which the robot will TURN to while driving.
     * @param finishedXMin the smallest X value that the drive will consider done at.
     * @param finishedXMax the largest X value that the drive will consider done at.
     * @param finishedYMin the smallest Y value that the drive will consider done at.
     * @param finishedYMax the largest Y value that the drive will consider done at.
     * @param finishedThetaMin the smallest Theta value that the drive will consider done at.
     * @param finishedThetaMax the largest Theta value that the drive will consider done at.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step. This is
     *         used/useful for stall outs.
     */
    public void pursuitDrive(double x, double y, double power, double theta,
                             double finishedXMin, double finishedXMax, double finishedYMin, double finishedYMax,
                             double finishedThetaMin, double finishedThetaMax, double timeoutS) {

        currentMethod = DRIVE_METHOD.PURE_PURSUIT;
        timeout = timeoutS;
        isDone = false;
        //targetX = x;
        //targetY = y;
        //strafePower = power;
        //targetTheta = theta;
        xMin = finishedXMin;
        xMax = finishedXMax;
        yMin = finishedYMin;
        yMax = finishedYMax;
        thetaMin = finishedThetaMin;
        thetaMax = finishedThetaMax;
        maxPower = power;

        isNonStop = false;
        //if the last drive was nonstop
        //updatesThread.powerUpdate.setNonStopTarget(x, y, power);

        // Reset timer once called
        runTime.reset();
    }
    public void turn(double theta, double timeoutS ){
        currentMethod = DRIVE_METHOD.TURN;
        timeout = timeoutS;
        //double currentTheta = realSense.getRotation();
        // while((theta - currentTheta) < -180){
        theta += 360;
    }
        /*while((theta - currentTheta) > 180){
            theta -= 360;
        }
        targetTheta = theta;
        clockwiseTurn = theta >  realSense.getRotation();
        runTime.reset();
        prevSec = runTime.seconds();
        prevzVal = realSense.getRotation();
        isNonStop = false;
        isDone = false;
    }*/

    /**
     * Used to move the robot across the field.  The robot can also TURN while moving along the path
     * in order for it to face a certain by the end of its path.  This method assumes the robot has
     * odometry modules which give an absolute position of the robot on the field.
     *
     *
     * @param power at which robot max speed can be set to using motion profiling.
     *
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void intakeDrive(double power, double timeoutS) {

        currentMethod = DRIVE_METHOD.INTAKE;
        timeout = timeoutS;
        isDone = false;
        strafePower = power;



        // Reset timer once called
        runTime.reset();
    }
    public double getDistance(){
        return (distanceSensor.getVoltage() - 0.16) * 83.3 + 10;

    }




    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
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
    /**
     * Sets powers to the four drive train motors.
     *
     * @param leftFront  motor's power.
     * @param rightFront motor's power.
     * @param leftBack   motor's power.
     * @param rightBack  motor's power.
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);
    }

    /**
     * Set drive train motors to BRAKE.
     */
    public void setDriveToBrake() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set drive train motors to FLOAT (coast).
     */
    public void setDriveToCoast() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Set drive train motors to STOP_AND_RESET_ENCODER.
     */
    public void resetDriveEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set drive train motors to RUN_USING_ENCODER.
     */
    public void setDriveRunUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set drive train motors to RUN_WITHOUT_ENCODER.
     */
    public void setDriveRunWithoutEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set drive train motors to RUN_TO_POSITION.
     */
    public void setDriveRunToPosition() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}