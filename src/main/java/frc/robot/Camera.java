package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.UsbCamera;

public class Camera {

    static UsbCamera cam0;
    static UsbCamera cam1;
    static VideoSink server;
    static boolean switched = false;

    static public void startCameras() {
        cam0 = CameraServer.startAutomaticCapture(0);
        cam0.setResolution(240,160);
        cam0.setFPS(10);
        cam1 = CameraServer.startAutomaticCapture(1);
        cam1.setResolution(240, 160);
        cam1.setFPS(10);
        server = CameraServer.getServer();
        server.setSource(cam0);
    }

    static public void changeCam() {

        switched = !switched;

        server.setSource(switched ? cam1 : cam0);
    }

} 