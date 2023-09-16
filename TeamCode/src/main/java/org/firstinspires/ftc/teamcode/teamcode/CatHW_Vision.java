package org.firstinspires.ftc.teamcode.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

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
import java.util.Collections;
import java.util.Deque;
import java.util.List;

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

        public static int regionWidth = 40;
        public static int regionHeight = 40;
        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(125,75);


        static int REGION_WIDTH = regionWidth;
        static int REGION_HEIGHT = regionHeight;

        final int FOUR_RING_THRESHOLD = 80;
        final int ONE_RING_THRESHOLD = 40;

        Point right_region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point right_region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth,
                REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        Mat Cr2 = new Mat();
        Mat Cr3 = new Mat();

        int avgRed;
        int avgBlue;
        int avgGreen;

        int avg1;
        int avg2;
        int avg3;

        Mat hsv = new Mat();
        Mat hsv2 = new Mat();
        Mat hsv3 = new Mat();
        Scalar redLowHSV1 = new Scalar(0, 100, 20); // lower bound HSV for red
        Scalar redHighHSV1 = new Scalar(10,255,255); // higher bound HSV for red

        Scalar redLowHSV2 = new Scalar(160, 100, 20); // lower bound HSV for red
        Scalar redHighHSV2 = new Scalar(179,255,255); // higher bound HSV for red

        Scalar blueLowHSV = new Scalar(100,150,0); // lower bound HSV for blue
        Scalar blueHighHSV = new Scalar(140,255,255); //higher bound HSV for blue

        Scalar greenLowHSV = new Scalar(36, 25, 25); // lower bound HSV for green
        Scalar greenHighHSV = new Scalar(70, 255,255); //higher bound HSV for green

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
            region1_Cb = Cr.submat(new Rect(right_region1_pointA, right_region1_pointB));
            region2_Cb = Cr2.submat(new Rect(right_region1_pointA, right_region1_pointB));
            region3_Cb = Cr3.submat(new Rect(right_region1_pointA, right_region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            right_region1_pointB.x = REGION1_TOPLEFT_ANCHOR_POINT.x + regionWidth;
            right_region1_pointB.y = REGION1_TOPLEFT_ANCHOR_POINT.y + regionHeight;

            inputToCb(input);

            Core.inRange(hsv, redLowHSV1, redHighHSV1,Cr);
            Core.inRange(hsv, redLowHSV2, redHighHSV2,Cr);
            avgRed = (int) Core.mean(region1_Cb).val[0];

            Core.inRange(hsv,blueLowHSV,blueHighHSV,Cr2);
            avgBlue = (int) Core.mean(region2_Cb).val[0];

            Core.inRange(hsv,greenLowHSV,greenHighHSV,Cr3);
            avgGreen = (int) Core.mean(region3_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    right_region1_pointA, // First point which defines the rectangle
                    right_region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = conePosition.NONE; // Record our analysis
            if(avgRed < avgBlue && avgBlue > avgGreen){
                position = conePosition.LEFT;

            }else if (avgBlue < avgRed && avgRed > avgGreen){
                position = conePosition.MIDDLE;
            }else if(avgGreen > avgRed && avgGreen > avgBlue){
                position = conePosition.RIGHT;
            }



            if (ringValues.size() > 29) {
                // Make sure we keep the size at a reasonable level
                ringValues.removeFirst();
            }
            ringValues.add(position);
            if (Collections.frequency(ringValues, conePosition.LEFT) > Collections.frequency(ringValues, conePosition.RIGHT) &&
                    Collections.frequency(ringValues, conePosition.LEFT) > Collections.frequency(ringValues, conePosition.MIDDLE)) {
                // If the amount of INSIDE readings is the most in the past 30 readings, return INSIDE.
                avgValue= conePosition.LEFT;
            } else if (Collections.frequency(ringValues, conePosition.RIGHT) > Collections.frequency(ringValues, conePosition.LEFT) &&
                    Collections.frequency(ringValues, conePosition.RIGHT) > Collections.frequency(ringValues, conePosition.MIDDLE)) {
                // If the amount of CENTER readings is the most in the past 30 readings, return CENTER.
                avgValue= conePosition.RIGHT;
            } else if (Collections.frequency(ringValues, conePosition.MIDDLE) > Collections.frequency(ringValues, conePosition.RIGHT) &&
                    Collections.frequency(ringValues, conePosition.MIDDLE) > Collections.frequency(ringValues, conePosition.LEFT)){
                // Just return back OUTSIDE since it is the last possible value.
               avgValue= conePosition.MIDDLE;
            }else{
                avgValue = conePosition.NONE;
            }

            return input;
        }
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, hsv3, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cr, 0);
            Core.extractChannel(hsv2, Cr2, 0);
            Core.extractChannel(hsv3, Cr3, 0);

        }



        public int avg1GetAnalysis() { return avg1; }
        public int avg2GetAnalysis(){ return avg2; }
        public int avg3GetAnalysis(){
            return avg3;
        }


    }



    UltimateGoalPipeline pipeline;
    OpenCvCamera webcam;


    private HardwareMap hwMap   = null;
    //private VuforiaLocalizer vuforia = null;
    //private VuforiaTrackables targetsUltimateGoal = null;
    //public List<VuforiaTrackable> allTrackables = null;
    private OpenGLMatrix lastLocation = null;
    double vuforiaX = 0;
    double vuforiaY = 0;
    boolean isVuforiaValid = false;
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch         = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;
    private static final float fullField  = 142 * mmPerInch;

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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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