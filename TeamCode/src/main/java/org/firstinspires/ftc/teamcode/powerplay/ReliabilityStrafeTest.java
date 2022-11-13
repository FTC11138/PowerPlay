package org.firstinspires.ftc.teamcode.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.baseBot.BaseAutonomousMethods;

@Autonomous(name = "ReliabilityStrafeTest", group = "Linear Opmode")
public class ReliabilityStrafeTest extends BaseAutonomousMethods{

    private ElapsedTime runtime = new ElapsedTime();
    private AutoAttachments robot = new AutoAttachments();

    @Override
    public void runOpMode() throws InterruptedException {

        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        sleep(1000);

        for (int i=0; i<5; i++) {
            encoderStraightDrive(20, 0.7);
            sleep(500);
            encoderStraightDrive(-20, 0.7);
            sleep(500);
        }

    }
}
