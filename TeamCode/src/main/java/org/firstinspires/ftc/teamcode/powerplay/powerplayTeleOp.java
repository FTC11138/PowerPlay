package org.firstinspires.ftc.teamcode.powerplay;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class powerplayTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments myRobot = new Attachments();

    private enum rotateD {
        STRAIGHT,
        LEFT90,
        RIGHT90,
        AROUND180,
        RANDOM
    }

    /* ------------------------------------- CONSTANTS ------------------------------------------ */
    private rotateD armDirection = rotateD.STRAIGHT;
    // Motors
    private double liftPower = 0;
    private int liftTarget = 0;
    private boolean useLiftPower = true;
    private double rotatePower = 0;
    private int rotateTarget = 0;
    private boolean useRotatePower = true;


    private boolean limits = true;
    private boolean shorten = false;

    private double currentLiftPosition = 0;
    private double currentSlidePosition = Constants.slideIn;
    private double currentRPosition = 0;

    // Servos
    private double slidePosition = Constants.slideIn;
    private double clawPosition = Constants.clawOpen;

    private int stage = -1;
    private int goalAngle = 0;
    private rotateD goalDirection = rotateD.STRAIGHT;

    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        myRobot.initialize(hardwareMap, telemetry);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        setRotationPosition(0.3, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /* ------------------------------------ Drive ------------------------------------ */
//        if (currentLiftPosition >= Constants.armStop){
        // Position constants
        currentLiftPosition = myRobot.getLiftMotorPosition();
        currentSlidePosition = myRobot.getSlidePosition();
        currentRPosition = myRobot.getRotationMotorPosition();

        // Motors
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double speedMultiplier = 1;
        double rotationMultiplier = 0.7;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.3;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.7;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 1;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 1;
        }

//        if (currentLiftPosition < -960/* && stage == -1*/) {
//            speedMultiplier = Constants.armDriveLimitRatio / currentLiftPosition;
//            rotationMultiplier = .72 * speedMultiplier;
//        }
//        if (ly > 0.1 && currentLiftPosition < -720) {
//            ly *= Constants.forwardLimitRatio;
//        }

//        telemetry.addData("Speed Multiplier", speedMultiplier);
//        telemetry.addData("Rotation Multiplier", rotationMultiplier);

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        myRobot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);
//        } else {
//            myRobot.runMotors(0, 0);
//        }





        /* ------------------------------------ Change ------------------------------------ */
        // Raising lift by power
        double liftJoystick = gamepad2.left_stick_y;
        if (liftJoystick > 0.12) {
            useLiftPower = true;
            liftPower = liftJoystick * Constants.liftDownRatio;
        } else if (liftJoystick < -0.12) {
            useLiftPower = true;
            liftPower = liftJoystick * Constants.liftRatio;
        } else if (useLiftPower) {
            liftPower = 0;
        }

        // Rotating stuff by power
        double armRJoystick = gamepad2.right_stick_x;
        if (Math.abs(armRJoystick) > 0.72) {
            useRotatePower = true;
            armDirection = rotateD.RANDOM;
            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rotatePower = armRJoystick * Constants.setRotateMultiplier;
        } else {
            rotatePower = 0;
        }

        // Stop powers
        if (gamepad1.left_bumper) {
            useRotatePower = true;
            rotatePower = 0;
            useLiftPower = true;
            liftPower = 0;
        }

        if (gamepad1.right_bumper) {
            limits = !limits;
        }

        /* ------------------------------------ Action ------------------------------------ */
        if (useLiftPower) {
            myRobot.runLiftMotor(liftPower);
        }
        if (useRotatePower) {
            myRobot.runRotateMotor(rotatePower);
        }

//        telemetry.addData("arm power: ", armPower);
//        telemetry.addData("use power: ", useArmPower);
        telemetry.addData("extension position", currentSlidePosition);
//        telemetry.addData("drop position", myRobot.dropServo.getPosition());
        telemetry.addData("lift position", currentLiftPosition);
//        telemetry.addData("lift power", useLiftPower);
        telemetry.addData("rotate position", currentRPosition);
        telemetry.addData("limits", limits);
        telemetry.update();

        Log.d("AHHHHHH extender", String.valueOf(currentSlidePosition));
        Log.d("AHHHHHH rotate", String.valueOf(currentRPosition));
        Log.d("AHHHHHH lift", String.valueOf(currentLiftPosition));
        Log.d("AHHHHH imu: ", "" + myRobot.getAngle());
    }

    @Override
    public void stop() {
    }

//    void setLiftMotor(int position, double tolerance, boolean usePID) {
//        dropPosition = Constants.dropClosePosition;
//        if (usePID) {
//            //Undefined constants
//            double newPower;
//            //Initial error
//            double error = (position - currentLiftPosition) / Constants.liftMax;
//            //Initial Time
//            telemetry.addData("1", "error: " + error);
//            if (Math.abs(error) > (tolerance / Constants.liftMax)) {
//                //Setting p action
//                newPower = Math.max(Math.min(error * Constants.liftkP, 1), -1);
////                Log.d("AHHHHHH liftMotor", "PID newPower: " + newPower);
////                telemetry.addData("liftMotor PID newPower", newPower);
//
//                //Set real power
//                newPower = Math.max(Math.abs(newPower), Constants.liftMin) * Math.signum(newPower);
//                if (Math.signum(newPower) == 1) {
//                    newPower = newPower * Constants.liftDownRatio;
//                }
//                myRobot.runLiftMotor(newPower);
//
//                //Logging
////                Log.d("AHHHHHH liftMotor", "error: " + (error * Constants.liftMax) + ", power: " + newPower + ", current position: " + currentPosition);
////                return false;
//            } else {
//                useLiftPower = true;
//                myRobot.runLiftMotor(0);
////                return true;
//            }
//        } else {
//            myRobot.setLiftMotor(1, position);
////            return true;
//        }
//    }

    public void setRotationPosition(double speed, int position) {
//        if ((currentLiftPosition < Constants.armSpin) ||
//                (currentSlidePosition > Constants.slideSpin) ||
//                Math.abs(position - currentRPosition) < 160) {
        myRobot.rotateMotor.setPower(speed);
        myRobot.rotateMotor.setTargetPosition(position);
        myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
    }

//    public void setRotationPositionPID(int position, double tolerance, boolean shorter) {
//        boolean spin = (currentLiftPosition <= Constants.armSpin);
//        if (spin) {
//            if (shorter) {
//                extPosition = Constants.extIn;
//            }
//            //Undefined constants
//            double newPower;
//            //Initial error
//            double error = (position - currentRPosition) / Constants.rotationMax;
//            //Initial Time
//            myRobot.rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if (Math.abs(error) > (tolerance / Constants.rotationMax)) {
//                //Setting p action
//                newPower = Math.max(Math.min(error * Constants.rotationkP, 1), -1);
//
//                //Set real power
//                newPower = Math.max(Math.abs(newPower), Constants.rotationMin) * Math.signum(newPower);
//                if (Math.signum(rotationPower) == 1) {
//                    if (currentRPosition > Constants.armRLimit) {
//                        newPower = 0;
//                        useRotationPower = true;
//                        rotationPower = 0;
//                    }
//                } else {
//                    if (currentRPosition < -Constants.armRLimit) {
//                        newPower = 0;
//                        useRotationPower = true;
//                        rotationPower = 0;
//                    }
//                }
//                myRobot.runRotateMotor(newPower);
//
//                //Logging
////                Log.d("AHHHHHH rotationMotor", "error: " + (error * Constants.rotationMax) + ", power: " + newPower + ", current position: " + currentRPosition);
////                return false;
//            } else {
//                rotationTarget = -1;
//                setRotationPosition(0.24, position);
////                if (hold) {
////
////                } else {
////                    useRotationPower = true;
////                    myRobot.runRotateMotor(0);
////                }
////                return true;
//            }
//        }/* else {
//            return false;
//        }*/
//    }
}
