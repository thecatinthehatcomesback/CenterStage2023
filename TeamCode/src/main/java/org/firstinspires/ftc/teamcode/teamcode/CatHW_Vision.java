package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayDeque;
import java.util.Deque;

/**
 * CatHW_Vision.java
 *
 *
 * A "hardware" class intended to contain common code for accessing camera and other vision related
 * situations.  While previous versions were made to mostly to test various forms of machine vision,
 * this version uses the Tensor Flow system from the FTC SDK to detect the SkyStones during init in
 * our autonomous routines. We've also tested Vuforia.  TODO:  Check if this is correct...
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Vision extends CatHW_Subsystem
{
    public static class UltimateGoalPipeline extends OpenCvPipeline
    {

        public static int regionWidth = 60;
        public static int regionHeight = 60;
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point RIGHT_REGION_TOPLEFT_ANCHOR_POINT = new Point(RobotConstants.rightRegionx,RobotConstants.rightRegiony);
        static final Point MIDDLE_REGION_TOPLEFT_ANCHOR_POINT = new Point(RobotConstants.middleRegionx,RobotConstants.middleRegiony);
        static final Point LEFT_REGION_TOPLEFT_ANCHOR_POINT = new Point(RobotConstants.leftRegionx,RobotConstants.leftRegiony);


        Point right_region_pointA = new Point(
                RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x,
                RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y);
        Point right_region_pointB = new Point(
                RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x + RobotConstants.rightRegionWidth,
                RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y + RobotConstants.rightRegionHeight);
        Point middle_region_pointA = new Point(
                MIDDLE_REGION_TOPLEFT_ANCHOR_POINT.x,
                MIDDLE_REGION_TOPLEFT_ANCHOR_POINT.y);
        Point middle_region_pointB = new Point(
                MIDDLE_REGION_TOPLEFT_ANCHOR_POINT.x + RobotConstants.middleRegionWidth,
                MIDDLE_REGION_TOPLEFT_ANCHOR_POINT.y + RobotConstants.middleRegionHeight);
        Point left_region_pointA = new Point(
                LEFT_REGION_TOPLEFT_ANCHOR_POINT.x,
                LEFT_REGION_TOPLEFT_ANCHOR_POINT.y);
        Point left_region_pointB = new Point(
                LEFT_REGION_TOPLEFT_ANCHOR_POINT.x + RobotConstants.leftRegionWidth,
                LEFT_REGION_TOPLEFT_ANCHOR_POINT.y + RobotConstants.leftRegionHeight);
        /*
         * Working variables
         */
        Mat right_Cb_red;
        Mat middle_Cb_red;
        Mat left_Cb_red;

        Mat right_Cb_blue;
        Mat middle_Cb_blue;
        Mat left_Cb_blue;

        /*
         * Working variables
         */
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cr2 = new Mat();
        Mat Cr3 = new Mat();

        int avgRightRed;
        int avgMiddleRed;
        int avgLeftRed;

        int avgRightBlue;
        int avgMiddleBlue;
        int avgLeftBlue;

        Mat hsv = new Mat();
        Mat hsv2 = new Mat();
        Mat hsv3 = new Mat();
        Scalar redLowHSV1 = new Scalar(0, 100, 20); // lower bound HSV for red
        Scalar redHighHSV1 = new Scalar(10,255,255); // higher bound HSV for red

        Scalar redLowHSV2 = new Scalar(160, 100, 20); // lower bound HSV for red
        Scalar redHighHSV2 = new Scalar(179,255,255); // higher bound HSV for red

        Scalar blueLowHSV = new Scalar(90, 50, 50);
        Scalar blueHighHSV = new Scalar(128, 255, 255);

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile conePosition position = conePosition.NONE;


        /* Enums */
        public enum conePosition
        {
            LEFT,
            MIDDLE,
            RIGHT,
            NONE,
        }
        private Deque<conePosition> ringValues;
        public conePosition avgValue = conePosition.NONE;

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            right_Cb_red = Cr.submat(new Rect(right_region_pointA, right_region_pointB));
            middle_Cb_red = Cr.submat(new Rect(middle_region_pointA, middle_region_pointB));
            left_Cb_red = Cr.submat(new Rect(left_region_pointA, left_region_pointB));

            right_Cb_blue = Cr2.submat(new Rect(right_region_pointA, right_region_pointB));
            middle_Cb_blue = Cr2.submat(new Rect(middle_region_pointA, middle_region_pointB));
            left_Cb_blue = Cr2.submat(new Rect(left_region_pointA, left_region_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            right_region_pointB.x = RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x + regionWidth;
            right_region_pointB.y = RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y + regionHeight;


            inputToCb(input);
            Mat mat = new Mat();

            Mat thresh = new Mat();

            Core.inRange(hsv, redLowHSV1, redHighHSV1,Cr);
            Core.inRange(hsv, redLowHSV2, redHighHSV2,Cr);

            avgRightRed = (int) Core.mean(right_Cb_red).val[0];

            avgMiddleRed = (int) Core.mean(middle_Cb_red).val[0];

            avgLeftRed = (int) Core.mean(left_Cb_red).val[0];
            Core.inRange(hsv,blueLowHSV,blueHighHSV,Cr2);
            avgRightBlue = (int) Core.mean(right_Cb_blue).val[0];

            avgMiddleBlue = (int) Core.mean(middle_Cb_blue).val[0];

            avgLeftBlue = (int) Core.mean(left_Cb_blue).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    right_region_pointA, // First point which defines the rectangle
                    right_region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    middle_region_pointA, // First point which defines the rectangle
                    middle_region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    left_region_pointA, // First point which defines the rectangle
                    left_region_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            if (ringValues.size() > 29) {
                // Make sure we keep the size at a reasonable level
                ringValues.removeFirst();
            }
            ringValues.add(position);
            position = conePosition.NONE; // Record our analysis
            if(avgRightRed > avgMiddleRed && avgRightRed > avgLeftRed ||
                    avgRightBlue > avgMiddleBlue && avgRightBlue > avgLeftBlue){
                position = conePosition.RIGHT;
            }else if (avgMiddleRed > avgRightRed && avgMiddleRed > avgLeftRed ||
                    avgMiddleBlue > avgRightBlue && avgMiddleBlue > avgLeftBlue){
                position = conePosition.MIDDLE;
            }else if(avgLeftRed > avgRightRed && avgLeftRed > avgMiddleRed ||
                    avgLeftBlue > avgRightBlue && avgLeftBlue > avgMiddleBlue){
                position = conePosition.LEFT;
            }

            return input;
        }
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cr, 0);
            Core.extractChannel(hsv, Cr2, 0);

        }



        public int avgRightGetAnalysis()  {  return avgRightRed;    }
        public int avgMiddleGetAnalysis() {  return avgMiddleRed;     }
        public int avgLeftGetAnalysis()    {  return avgLeftRed;    }


    }



    UltimateGoalPipeline pipeline;
    OpenCvCamera webcam;


    private HardwareMap hwMap   = null;

    /* Constructor */
    public CatHW_Vision(CatHW_Async mainHardware){
        super(mainHardware);
    }

    /**
     * Initializes the "hardware" devices for anything having to do with machine vision.
     *
     * @param ahwMap which contains the hardware to look for.
     */
    public void initVision(HardwareMap ahwMap) {

        hwMap = ahwMap;

        mainHW.opMode.telemetry.addData("Initializing","Vision 1");
        mainHW.opMode.telemetry.update();
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        mainHW.opMode.telemetry.addData("Initializing","Vision 2");
        mainHW.opMode.telemetry.update();
        pipeline = new UltimateGoalPipeline();
        pipeline.ringValues = new ArrayDeque<>(30);
        mainHW.opMode.telemetry.addData("Initializing","Vision 3");
        mainHW.opMode.telemetry.update();

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1024, 768, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {

            }

        });

    }
    public void stop(){
        if(webcam != null){webcam.closeCameraDevice();}
        //if(targetsUltimateGoal != null){targetsUltimateGoal.deactivate();}

    }
    public UltimateGoalPipeline.conePosition getConePos(){
        return pipeline.position;
    }



}