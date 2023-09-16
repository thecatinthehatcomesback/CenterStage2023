package org.firstinspires.ftc.teamcode.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.spartronics4915.lib.T265Camera;

public class CatHW_RealSense extends CatHW_Subsystem {
    /* Constructor */
    public CatHW_RealSense(CatHW_Async mainHardware){
        super(mainHardware);
    }
    CatHW_Async robot = new CatHW_Async();
    private static T265Camera slamra = null;
    private static boolean isCamStart = false;

    public double xPos;
    public double yPos;

    double lastAngle;
    double currentAngle;

    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hwMap.appContext);

        }
        if(!isCamStart) {
            slamra.start();
            slamra.setPose(new Pose2d(0,0,new Rotation2d(0)));
            isCamStart = true;
        }else{
            slamra.setPose(new Pose2d(0,0,new Rotation2d(0)));
        }
        if(slamra.isStarted()){
            Log.d("catbot","T265 is Started");
        }else{
            Log.d("catbot","T265 is NOT Started");
        }

    }

    public void resetPos(){
        slamra.setPose(new Pose2d(0,0,new Rotation2d(0)));
    }

    public boolean getCameraUpdate(){
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return false;

        // We divide by 0.0254 to convert meters to inches

        if(robot.isLeftAlliance){
            xPos = -up.pose.getTranslation().getX() / 0.0254;
            yPos = -up.pose.getTranslation().getY() / 0.0254;
            updateCurrentAngle(up.pose.getRotation());
        }else if (!robot.isLeftAlliance){
            xPos = up.pose.getTranslation().getX() / 0.0254;
            yPos = up.pose.getTranslation().getY() / 0.0254;
            updateCurrentAngle(up.pose.getRotation());
        }

        return true;

    }

    public double getRotation(){
        return currentAngle;
    }

    public double getRotationRadians(){
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        return up.pose.getHeading();
    }

    public double getXPos(){
        return xPos;
    }

    public double getYPos() {
        return yPos;
    }

    public void updateCurrentAngle(Rotation2d rotation) {

        double newAngle = -rotation.getDegrees();

        if((lastAngle > 90) && (newAngle < -90)){
            currentAngle = currentAngle+(newAngle + 360)-reduceAngle(currentAngle);
        }else if((lastAngle < -90) && (newAngle > 90)){
            currentAngle = currentAngle+(newAngle - 360)-reduceAngle(currentAngle);

        }else{
            currentAngle = currentAngle +(newAngle)-reduceAngle(currentAngle);
        }
        lastAngle = newAngle;

    }
    //returns an angle reduced to a range of -180 to +180
    public double reduceAngle(double angle){
        while((angle > 180) || (angle < -180)){
            if(angle > 180){
                angle = angle - 360;
            }else if(angle < -180){
                angle = angle + 360;
            }
        }
        return angle;
    }
}