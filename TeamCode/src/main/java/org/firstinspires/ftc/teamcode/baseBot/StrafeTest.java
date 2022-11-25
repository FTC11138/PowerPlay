package org.firstinspires.ftc.teamcode.baseBot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "StrafeTest", group = "Linear Opmode")

public class StrafeTest extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    View relativeLayout;

    @Override
    public void runOpMode() {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        waitForStart();

        runtime.reset();

        sleep(120);

        myRobot.lb.setPower(-Constants.ColorStripAlignmentSpeed);
        myRobot.lf.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rb.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rf.setPower(-Constants.ColorStripAlignmentSpeed);
        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            try {
                if (runSample()) {
                    myRobot.lb.setPower(0);
                    myRobot.lf.setPower(0);
                    myRobot.rb.setPower(0);
                    myRobot.rf.setPower(0);
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

        sleep(Constants.ColorStripAlignmentDelay);

        myRobot.lb.setPower(0.2);
        myRobot.lf.setPower(-0.2);
        myRobot.rb.setPower(-0.2);
        myRobot.rf.setPower(0.2);
        while (true) {
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

            try {
                if (runSample()) {
                    myRobot.lb.setPower(0);
                    myRobot.lf.setPower(0);
                    myRobot.rb.setPower(0);
                    myRobot.rf.setPower(0);
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

    private boolean runSample() {
        float gain = 30;
        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        waitForStart();

        colorSensor1.setGain(gain);
        colorSensor2.setGain(gain);

        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        Color.colorToHSV(colors1.toColor(), hsvValues1);
        Color.colorToHSV(colors2.toColor(), hsvValues2);

        if (colors1.blue > Constants.ColorThresh && colors2.blue > Constants.ColorThresh) {
            return true;
        } else {
            return false;
        }
    }
}