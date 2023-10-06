/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@TeleOp
public class EasyOpenCVExample extends LinearOpMode
{
    static RobotConstants pos = new RobotConstants();
    //OpenCvInternalCamera phoneCam;
    CenterstageDeterminationPipeline pipeline;
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CenterstageDeterminationPipeline();
        webcam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);  doesn't work with webcam

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addData("HERE", "");
        telemetry.update();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis Right", pipeline.avgRightGetAnalysis());
            telemetry.addData("Analysis Middle", pipeline.avgMiddleGetAnalysis());
            telemetry.addData("Analysis Left", pipeline.avgLeftGetAnalysis());



            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            dashboardTelemetry.addData("Analysis Red", pipeline.avgRightGetAnalysis());
            dashboardTelemetry.addData("robot pos", pipeline.robotPosition);
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    @Config
    public static class CenterstageDeterminationPipeline extends OpenCvPipeline
    {

        public static int regionWidth = 60;
        public static int regionHeight = 60;

        /*
         * An enum to define the ring position
         */
        public enum conePosition {
            LEFT,
            MIDDLE,
            RIGHT,
            NONE,
        }

        public  enum robotPos{
            LEFT,
            RIGHT,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */

        //TODO: change point cordinates to correct position
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
        Mat right_Cb;
        Mat middle_Cb;
        Mat left_Cb;


        Mat YCrCb = new Mat();
        Mat Cr = new Mat();

        int avgRight;
        int avgMiddle;
        int avgLeft;

        Mat hsv = new Mat();
        Scalar redLowHSV1 = new Scalar(0, 100, 20); // lower bound HSV for red
        Scalar redHighHSV1 = new Scalar(10,255,255); // higher bound HSV for red

        Scalar redLowHSV2 = new Scalar(160, 100, 20); // lower bound HSV for red
        Scalar redHighHSV2 = new Scalar(179,255,255); // higher bound HSV for red





        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile conePosition position = conePosition.NONE;
        private volatile robotPos robotPosition = robotPos.NONE;



        /*
         * This function takes the RGB frame, converts to HSV,
         * and extracts the Cr channel to the 'Cr' variable
         */

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cr, 0);

        }



        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            right_Cb = Cr.submat(new Rect(right_region_pointA, right_region_pointB));
            middle_Cb = Cr.submat(new Rect(middle_region_pointA, middle_region_pointB));
            left_Cb = Cr.submat(new Rect(left_region_pointA, left_region_pointB));
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
            avgRight  = (int) Core.mean(right_Cb).val[0];

            avgMiddle = (int) Core.mean(middle_Cb).val[0];

            avgLeft   = (int) Core.mean(left_Cb).val[0];




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

            position = conePosition.NONE; // Record our analysis
            if(avgRight > avgMiddle && avgRight > avgLeft){
                position = conePosition.RIGHT;
            }else if (avgMiddle > avgRight && avgMiddle > avgLeft){
                position = conePosition.MIDDLE;
            }else if(avgLeft > avgRight && avgLeft > avgMiddle){
                position = conePosition.LEFT;
            }

            return input;
        }

        public int avgRightGetAnalysis()  {  return avgRight;    }
        public int avgMiddleGetAnalysis() {  return avgMiddle;     }
        public int avgLeftGetAnalysis()    {  return avgLeft;    }




    }
}