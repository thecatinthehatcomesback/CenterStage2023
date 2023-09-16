package org.firstinspires.ftc.teamcode.teamcode;
import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp
public class TestVision extends LinearOpMode {
    OpenCvCamera webcam;

    public enum conePosition {
        FIRST,
        SECOND,
        THIRD,
        NONE,
    }

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SamplePipeline pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
                //FtcDashboard.getInstance().startCameraStream(webcam, 10);

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
            telemetry.addData("Decoded data: ", pipeline.output);
            telemetry.addData("Position ", pipeline.position);

            telemetry.update();
            dashboardTelemetry.addData("Decoded Data: ", pipeline.data);
            dashboardTelemetry.addData("Position ", pipeline.position);
            dashboardTelemetry.addData("Process Counter: ",pipeline.processCount);
            dashboardTelemetry.addData("QR Code Detected? ",pipeline.detected);
            dashboardTelemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

        }
    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        QRCodeDetector decoder = new QRCodeDetector();
        String output = null;
        conePosition position = conePosition.NONE;
        int processCount = 0;
        boolean detected = false;
        String data = null;
        @Override
        public void init(Mat firstFrame)
        {
            Mat inputBGR = new Mat();
            Imgproc.cvtColor(firstFrame, inputBGR, Imgproc.COLOR_RGB2GRAY);

        }

        @Override
        public Mat processFrame(Mat input)
        {
            processCount += 1;
            Mat inputBGR = new Mat();
            Mat points = new Mat();
            Rect area = new Rect(480,270,180,180);
            Mat cropped = new Mat(input,area);
            Imgproc.cvtColor(cropped, inputBGR, Imgproc.COLOR_RGB2GRAY);

            Bitmap bmp = Bitmap.createBitmap(inputBGR.cols(),inputBGR.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(inputBGR, bmp);

            FtcDashboard.getInstance().sendImage(bmp);
            data = decoder.detectAndDecodeCurved(inputBGR, points);
            detected = decoder.detect(inputBGR, points);

            if(data.equals("https://www.youtube.com/watch?v=dQw4w9WgXcQ&ab_channel=RickAstley")){
                output = data;
                position = conePosition.FIRST;
            } else if(data.equals("https://www.youtube.com/c/FTCTheCatintheHatComesBack")){
                output = data;
                position = conePosition.SECOND;
            } else if(data.equals("https://www.shercofirst.org/")){
                output = data;
                position = conePosition.THIRD;
            }

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }

}
