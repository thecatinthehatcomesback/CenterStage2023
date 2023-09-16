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
    //OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    OpenCvCamera webcam;

    @Override
    public void runOpMode()
    {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
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
            telemetry.addData("Analysis Right", pipeline.avg1RedGetAnalysis());
            telemetry.addData("Analysis Middle", pipeline.avg2BlueAnalysis());
            telemetry.addData("Analysis Left", pipeline.avg3GreenAnalysis());
            telemetry.addData("Analysis","Top Left: %d Top Right: %d", pipeline.getTopLeftAvg(),pipeline.getTopRightAvg());



            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            dashboardTelemetry.addData("Analysis Red", pipeline.avg1RedGetAnalysis());
            dashboardTelemetry.addData("Analysis Blue", pipeline.avgBlue);
            dashboardTelemetry.addData("Analysis Green", pipeline.avgGreen);
            dashboardTelemetry.addData("Analysis","Top Left: %d Top Right: %d", pipeline.getTopLeftAvg(),pipeline.getTopRightAvg());
            dashboardTelemetry.addData("robot pos", pipeline.robotPosition);
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    @Config
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100,100);



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

        int topLeftAvg;
        int topRightAvg;

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
        private volatile robotPos robotPosition = robotPos.NONE;



        /*
         * This function takes the RGB frame, converts to HSV,
         * and extracts the Cr channel to the 'Cr' variable
         */

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, hsv2, Imgproc.COLOR_RGB2HSV);
            Imgproc.cvtColor(input, hsv3, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, Cr, 0);
            Core.extractChannel(hsv2, Cr2, 0);
            Core.extractChannel(hsv3, Cr3, 0);

        }



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
            Mat mat = new Mat();

            Mat thresh = new Mat();

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
            if(avgRed > avgBlue && avgRed > avgGreen){
                position = conePosition.RIGHT;
            }else if (avgBlue > avgRed && avgBlue > avgGreen){
                position = conePosition.MIDDLE;
            }else if(avgGreen > avgRed && avgGreen > avgBlue){
                position = conePosition.LEFT;
            }

            robotPosition = robotPos.NONE;
            if(topLeftAvg > topRightAvg){
                robotPosition = robotPos.RIGHT;
            }else if(topRightAvg > topLeftAvg){
                robotPosition = robotPos.LEFT;
            }

            return input;
        }

        public int avg1RedGetAnalysis() {  return avgRed;      }
        public int avg2BlueAnalysis()   {  return avgBlue;     }
        public int avg3GreenAnalysis()  {  return avgGreen;    }
        public int getTopLeftAvg()      {  return topLeftAvg;  }
        public int getTopRightAvg()     {  return topRightAvg; }




    }
}