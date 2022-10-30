package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

class SignalDetectionPipeline extends OpenCvPipeline {

    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    int counter;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Y channel to the 'Y' variable
     */
    void inputToY(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        ArrayList<Mat> yCrCbChannels = new ArrayList(3);
        Core.split(YCrCb, yCrCbChannels);
        Y = yCrCbChannels.get(0);
    }

    int detectBlackWhite(Mat input) {
        int edgeCounter = 0;
        int x1 = Constants.leftBoundary;
        int x2 = Constants.rightBoundary;
        int midy = Constants.middleLine;
        for (int j = x1 + 1; j < x2; j++) {
//            if (input.at(Byte.class, (y2 + y1) / 2, j).getV4c().get_0() - input.at(Byte.class, (y2 + y1) / 2, j - 1).getV4c().get_0() > Constants.changeThresh) {
            if ((input.at(Byte.class, midy, j).getV().byteValue() - input.at(Byte.class, midy, j - 1).getV().byteValue()) > Constants.changeThresh) {
                edgeCounter += 1;
            }
        }
        if (edgeCounter == 0) { // clip
            edgeCounter = 1;
        } else if (edgeCounter > 3) {
            edgeCounter = 3;
        }
        return edgeCounter;
    }

    @Override
    public void init(Mat firstFrame) {
        inputToY(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToY(input);
        Y.copyTo(input);
        counter = detectBlackWhite(input);
        YCrCb.release(); // don't leak memory!
        Y.release(); // don't leak memory!

        return input;
    }

    public int getCounter(){
        return counter;
    }

}
@Autonomous(name = "BlueRight", group = "Linear Opmode")
public class BlueRight extends BaseAutonomousMethods{

    private ElapsedTime runtime = new ElapsedTime();

    OpenCvWebcam webcam;
    org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline signalDetectionPipeline;
    int signal = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    /*
    @Override
    public void loop() {
//        telemetry.addData("Image Analysis:",pipeline.getAnalysis());
//        telemetry.addData("Image number:", pipeline.getRectA_Analysis());
        telemetry.addData("Image number:", signalDetectionPipeline.getCounter());
        telemetry.update();

        TelemetryPacket pack = new TelemetryPacket();
        pack.put("Image number:", signalDetectionPipeline.getCounter());
        dashboard.sendTelemetryPacket(pack);
    }*/

    /*
     * Code to run ONCE when the driver hits INIT
     */
    /*@Override
    public void init() {
        telemetry.addData("Status", "Initialized");


    }*/

    @Override
    public void runOpMode() throws InterruptedException {
       // telemetry.addData("Status", "Initialized");
       // telemetry.update();
        initializeAutonomousDrivetrain(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new org.firstinspires.ftc.teamcode.powerplay.SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Tell the driver that initialization is complete.
        telemetry.addData("Signal", signal);
        telemetry.update();

        waitForStart();
        runtime.reset();


        sleep(120);

        //signal = signalDetectionPipeline.getCounter();

        telemetry.addData("Signal", signal);
        telemetry.update();

        encoderStraightDrive(36, 0.5);
        // encoderStrafeDriveInchesRight(12, 0.5);

        do {
            signal = signalDetectionPipeline.getCounter();
            if (runtime.seconds() >= 10) {
                if (signal == 0) {
                    encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                } else if (signal >= 3) {
                    encoderStrafeDriveInchesRight(18, 0.5); // turn right
                } else if (signal == 1) {
                    encoderStraightDrive(36, 0.5);
                } else if (signal == 2) {
                    encoderStraightDrive(-36, 0.5);
                }

                sleep(2000);
                // return to original
                /*if (signal == 1) {
                    encoderStrafeDriveInchesRight(18, 0.5); // turn right
                } else if (signal == 3) {
                    encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                }
                encoderStraightDrive(-36, 0.5);*/
                break;
            }
        } while (true);
    }
}
