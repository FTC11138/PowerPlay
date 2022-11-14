package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Autonomous(name = "RightAutoWithColorSens", group = "Linear Opmode")
public class RightAutoWithColorSens extends BaseAutonomousMethods{

    private ElapsedTime runtime = new ElapsedTime();
    private AutoAttachments robot = new AutoAttachments();

    View relativeLayout;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);

        robot.colorSensor.setGain(Constants.gain);

        double originalAngle = 0;
        double currentAngle;
        double angleError;
        
        waitForStart();
        runtime.reset();

        encoderStraightDrive(3, 0.2);
        encoderStrafeDriveInchesRight(18, 0.5);
        encoderStraightDrive(25, 0.7);

        runMotors(0.2, 0.2);

        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - originalAngle);
            runMotors(0.2 + angleError * Constants.tkR, 0.2 - angleError * Constants.tkR);

            try {
                if (calculateCurrentColor()) {
                    runMotors(0, 0);
                    break;
                }
            } finally {
                relativeLayout.post(new Runnable() {
                    public void run() {
                        relativeLayout.setBackgroundColor(Color.WHITE);
                    }
                });
            }

        }

    }

    private boolean calculateCurrentColor() {

        final float[] hsvValues = new float[3];

        NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red 1", "%.1f", colors.red)
                .addData("Green 1", "%.1f", colors.green)
                .addData("Blue 1", "%.1f", colors.blue);

        telemetry.update();

        if (colors.blue > Constants.ColorThresh) {
            return true;
        } else {
            return false;
        }

    }
}
