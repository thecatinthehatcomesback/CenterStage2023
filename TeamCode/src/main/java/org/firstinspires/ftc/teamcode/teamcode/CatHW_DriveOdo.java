package org.firstinspires.ftc.teamcode.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.drive.CatMecanumDrive;

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

    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;

    // Motors:
    //public DcMotor leftFrontMotor = null;
   // public DcMotor rightFrontMotor = null;
    //public DcMotor leftRearMotor = null;
    //public DcMotor rightRearMotor = null;

    //public AnalogInput distanceSensor = null;



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
    CatMecanumDrive drive;

    /**
     * Initialize standard Hardware interfaces in the CatHW_DriveOdo hardware.
     *
     * @throws InterruptedException in case of error.
     */
    public void init()  throws InterruptedException  {

        drive = new CatMecanumDrive(hwMap);
        leftDistance = hardwareMap.get(DistanceSensor.class, "leftDistance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "rightDistance");

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
    /*public double getDistance(){
        return (distanceSensor.getVoltage() - 0.16) * 83.3 + 10;

    }*/




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


    public void scoreHex(){
        double theta = Math.atan((rightDistance.getDistance(DistanceUnit.INCH)-
                leftDistance.getDistance(DistanceUnit.INCH))/(14+10));
    }
}