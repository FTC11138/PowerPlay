package org.firstinspires.ftc.teamcode.powerplay;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.baseBot.Drivetrain;

public class Attachments extends Drivetrain {
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx liftMotor, rotateMotor;
    public Servo clawServo, slideServo; //, camServo;
    public Rev2mDistanceSensor rightDistance, leftDistance, clawRightDistance, clawLeftDistance, clawDistance;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        telemetry = telemetry_;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Motors
        liftMotor = hardwareMap.get(DcMotorEx.class, names.liftMotor);
        rotateMotor = hardwareMap.get(DcMotorEx.class, names.rotateMotor);

        // Servos
        clawServo = hardwareMap.get(Servo.class, names.clawServo);
        slideServo = hardwareMap.get(Servo.class, names.slideServo);
//        camServo = hardwareMap.get(Servo.class, names.camServo);

        // Sensors
//        leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.leftDistance);
//        rightDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.rightDistance);
        clawDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.clawDistance);
//        clawLeftDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.clawLeftDistance);
//        clawRightDistance = hardwareMap.get(Rev2mDistanceSensor.class, names.clawRightDistance);

        // Motor Behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initializeDriveTrain(hardwareMap, telemetry_);
    }

    /* --------------------------------------- ACCESSORS --------------------------------------- */
//    public double getRightDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getLeftDistance() {return rightDistance.getDistance(DistanceUnit.INCH);}
//    public double getClawRightDistance() {return clawRightDistance.getDistance(DistanceUnit.INCH)}
//    public double getClawLeftDistance() {return clawLeftDistance.getDistance(DistanceUnit.INCH)}
    public double getClawDistance() {return clawDistance.getDistance(DistanceUnit.INCH);}

    /* ---------------------------------------- SETTERS ---------------------------------------- */
    public void runLiftMotor(double power) {
        liftMotor.setPower(power);
    }
    public void setLiftMotor(double power, int position) {
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runRotateMotor(double power) {rotateMotor.setPower(power);}
    public void setRotateMotor(double power, int position) {
        rotateMotor.setPower(power);
        rotateMotor.setTargetPosition(position);
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setClawServo (double position) {clawServo.setPosition(position);}
//    public void setCamServo (double position) {camServo.setPosition(position);}
    public void setSlideServo (double position) {slideServo.setPosition(position);}

    public int getLiftMotorPosition() {
        return liftMotor.getCurrentPosition();
    }

    public double getSlidePosition() {
        return slideServo.getPosition();
    }

    public int getRotationMotorPosition() {
        return rotateMotor.getCurrentPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }
}
