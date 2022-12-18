package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplay.Attachments;
import org.firstinspires.ftc.teamcode.powerplay.Constants;

@TeleOp
@Config
@Disabled
public class LiftPIDTuner extends OpMode {
    Attachments myRobot = new Attachments();
    private PIDController controller;
    public static double kp = Constants.liftkP;
    public static double ki = Constants.liftkI;
    public static double kd = Constants.liftkD;
    public static double kf = Constants.liftkF; // 0.1
    public static int target = -2750;

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        controller = new PIDController(kp, ki, kd);
        controller.setTolerance(Constants.liftTolerance);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // -2750, -1000
        controller.setPID(kp, ki, kd);
        controller.setTolerance(Constants.liftTolerance);
        int currentLiftPos = myRobot.getLiftMotorPosition();
        double pid = controller.calculate(currentLiftPos, target);
        double pow = pid + kf * currentLiftPos / -Constants.liftMax;

        myRobot.runLiftMotor(pow);
        telemetry.addData("pos", currentLiftPos);
        telemetry.addData("target", target);
        telemetry.addData("power", pow);
        telemetry.addData("error", controller.getPositionError());
//        telemetry.addData("pow", pid);
        telemetry.update();
    }
}
