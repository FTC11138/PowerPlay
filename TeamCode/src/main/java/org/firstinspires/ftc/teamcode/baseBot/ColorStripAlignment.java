package org.firstinspires.ftc.teamcode.baseBot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "ColorStripAlignment", group = "Linear Opmode")
@Disabled
public class ColorStripAlignment extends BaseAutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    View relativeLayout;

    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    private void runSample() {
        float gain = 30;
        final float[] hsvValues1 = new float[3];
        final float[] hsvValues2 = new float[3];

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color2");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);

            colorSensor1.setGain(gain);
            colorSensor2.setGain(gain);

            NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
            NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues1);
            Color.colorToHSV(colors2.toColor(), hsvValues2);

            telemetry.addLine()
                    .addData("Red 1", "%.1f", colors1.red)
                    .addData("Green 1", "%.1f", colors1.green)
                    .addData("Blue 1", "%.1f", colors1.blue);
            telemetry.addLine()
                    .addData("Red 2", "%.1f", colors2.red)
                    .addData("Green 2", "%.1f", colors2.green)
                    .addData("Blue 2", "%.1f", colors2.blue);
            if (colors1.blue > Constants.ColorThresh && colors2.blue > Constants.ColorThresh) {
                telemetry.addLine("ALIGNED");
            }
            telemetry.update();
        }
    }
}