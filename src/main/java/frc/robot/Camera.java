package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;

public class Camera {

    static UsbCamera cam0;
    static UsbCamera cam1;
    static VideoSink server;
    static boolean switched = false;

    static public void startCameras() {
        cam0 = CameraServer.getInstance().startAutomaticCapture(0);
        //cam0.setResolution(240,160);
        //cam0.setFPS(15);
        cam1 = CameraServer.getInstance().startAutomaticCapture(1);
        //cam1.setResolution(240, 160);
        //cam1.setFPS(15);
        server = CameraServer.getInstance().getServer();
        server.setSource(cam0);
    }

    static public void changeCam() {

        switched = !switched;

        server.setSource(switched ? cam1 : cam0);
    }

} 