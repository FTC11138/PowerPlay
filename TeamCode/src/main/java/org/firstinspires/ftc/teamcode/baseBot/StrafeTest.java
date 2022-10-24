package org.firstinspires.ftc.teamcode.baseBot;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@Autonomous(name = "StrafeTest", group = "Linear Opmode")

public class StrafeTest extends BaseAutonomousMethods {

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

        double originalAngle = getHorizontalAngle();
        double currentAngle;
        double angleError;

        myRobot.lb.setPower(-Constants.ColorStripAlignmentSpeed);
        myRobot.lf.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rb.setPower(Constants.ColorStripAlignmentSpeed);
        myRobot.rf.setPower(-Constants.ColorStripAlignmentSpeed);
        while (true) {

            currentAngle = getHorizontalAngle();
            angleError = loopAround(currentAngle - originalAngle);
            myRobot.lb.setPower(-Constants.ColorStripAlignmentSpeed + angleError * Constants.tkR);
            myRobot.lf.setPower(Constants.ColorStripAlignmentSpeed + angleError * Constants.tkR);
            myRobot.rb.setPower(Constants.ColorStripAlignmentSpeed - angleError * Constants.tkR);
            myRobot.rf.setPower(-Constants.ColorStripAlignmentSpeed - angleError * Constants.tkR);

        }
    }
}