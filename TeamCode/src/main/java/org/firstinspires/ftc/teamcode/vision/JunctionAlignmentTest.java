package org.firstinspires.ftc.teamcode.vision;

////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Config
@Autonomous(group = "Vision")
public class JunctionAlignmentTest extends LinearOpMode {

    OpenCvWebcam camera;
    JunctionAlignment pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize camera and servow
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "backWebcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pipeline = new JunctionAlignment(telemetry);

        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        while (opModeInInit()) {
            if (gamepad1.a) {
                while (gamepad1.a) {
                }
                pipeline.normalizeToPole(70, 5);
                sleep(2000);
            }


            waitForStart();


            camera.closeCameraDevice();
        }
    }
}