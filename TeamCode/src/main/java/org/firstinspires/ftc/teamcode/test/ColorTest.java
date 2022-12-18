package org.firstinspires.ftc.teamcode.test;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;

import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "ColorTest", group = "Linear Opmode")
@Disabled
public class ColorTest extends AutonomousMethods {

    private ElapsedTime runtime = new ElapsedTime();
    NormalizedColorSensor colorSensor;
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
        float gain = Constants.gain;
        final float[] hsvValues = new float[3];

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);

            colorSensor.setGain(Constants.gain);

            NormalizedRGBA colors1 = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors1.toColor(), hsvValues);

            telemetry.addLine()
                    .addData("Red 1", "%.1f", colors1.red)
                    .addData("Green 1", "%.1f", colors1.green)
                    .addData("Blue 1", "%.1f", colors1.blue);
            if (colors1.red > Constants.RedThresh || colors1.blue > Constants.BlueThresh) {
                telemetry.addLine("ALIGNED");
            }
            telemetry.update();
        }
    }
}