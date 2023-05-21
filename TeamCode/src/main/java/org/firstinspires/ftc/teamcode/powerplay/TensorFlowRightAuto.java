package org.firstinspires.ftc.teamcode.powerplay;
/**
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.powerplay.AutonomousMethods;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
 */
/**
@Autonomous(name = "TensorFlowRightAuto", group = "Linear Opmode")
@Disabled
public class TensorFlowRightAuto extends AutonomousMethods{
    final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/lite-model_efficientdet_lite0_detection_metadata_1.tflite";
    final String[] LABELS = {
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "street sign",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "hat",
            "backpack",
            "umbrella",
            "shoe",
            "eye glasses",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "plate",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "mirror",
            "dining table",
            "window",
            "desk",
            "toilet",
            "door",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "blender",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
            "hair brush"
    };

    final String VUFORIA_KEY =
            "AUpeoKP/////AAABmXarRDwQxEWbjqbezmXHfkwA5aEeXNOsiGD31T/itLYty7SNnzDsX5pZm29e0d1DBEKl8c83JY6KumPJUWuHV+pfQArB21d2OVNaAZVw+wozaEQqTkFp1iPNuwDF3pk38unJDYfdG/EIpJrMQS48F3+xs80hhfhRs+6GoTxb3VxuLPxWKn8ZqRLzjsvjDjNo6hLKmqxRPUoKRK81Em/vzL9Wc3Q4MhFHRwNF+lMfu1CbxCOS0NYLjo0cIPTeICgdZ5XPy0gBfSgrmsSi21gcKCiIvnLbKQjDYiBkdqE14kZFRsfQD4QhmRIFt8kGznQ78e4ufC/B7ywJ1HlRystIVjy2l6OhLh4dQbfcND3jNjmL";

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;


    //static final int STREAM_WIDTH = 1920; // modify for your camera
    //static final int STREAM_HEIGHT = 1080; // modify for your camera

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

    /**
    OpenCvWebcam webcam;
    SignalDetectionPipeline signalDetectionPipeline;
    int signal = 1;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    */
 /**
    @Override
    public void runOpMode() throws InterruptedException {telemetry.update();
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        robot.initialize(hardwareMap, telemetry);

    /**
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        signalDetectionPipeline = new SignalDetectionPipeline();
        webcam.setPipeline(signalDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        }); */

 /**
        waitForStart();
        runtime.reset();

        initVuforia();
        initTfod();

        org.firstinspires.ftc.teamcode.powerplay.FunctionTFODEverydayObjects getLabel;
        telemetry.addData("Function Start", "true");
        telemetry.update();
        encoderStraightDrive(Constants.moveForward, 0.2);
        getLabel = new org.firstinspires.ftc.teamcode.powerplay.FunctionTFODEverydayObjects();
        String finalLabel = getLabel.getTFODLabel();
        telemetry.addData("Function stop", "true");
        telemetry.update();
        //sleep(2000);

        /**signal = signalDetectionPipeline.getCounter();
        telemetry.addData("Signal", signal);
        telemetry.update(); */
        /**
        robot.setClawServo(Constants.clawOpen);
        sleep(500);
        robot.setLiftMotor(0.1, -100);
        sleep(500);

        encoderStraightDrive(3, 0.2);
        encoderStrafeDriveInchesRight(3, 0.5);

        robot.setLiftMotor(0.3, Constants.liftLow);

        encoderStraightDrive(33, 0.5);
        sleep(500);

        robot.setLiftMotor(0.5, Constants.liftHigh);
        robot.setRotateMotor(0.5, -40 * Constants.rotMotorPosPerDegree);
        sleep(2000);
        robot.setSlideServo(0.33);
        sleep(2000);
        robot.setLiftMotor(0.3, Constants.liftHigh + 200);
        sleep(1000);
        robot.setClawServo(Constants.clawClose);
        sleep(1000);
        robot.setSlideServo(Constants.slideIn);
        robot.setRotateMotor(0.5, 0);

        encoderStraightDrive(4, 0.5); // push signal cone ahead
        sleep(500);

        encoderStraightDrive(-4, 0.5); // move back
        sleep(500);

        robot.setLiftMotor(0.3, 0);
        sleep(3000);


        do {
            if (runtime.seconds() >= 5) {
                if (finalLabel == "stop sign") {
                    encoderStrafeDriveInchesRight(-18, 0.5); // turn left
                } else if (finalLabel == "car") {
                    encoderStrafeDriveInchesRight(18, 0.5); // turn right
                }
                sleep(3000);

//                if (Constants.debugMode) {
//                    sleep(2000);
//                    encoderTurn(0, 0.5, 5);
//                    // return to original, for testing purpose. REMOVE IT before competition!!!!
//                    if (signal == 1) {
//                        encoderStrafeDriveInchesRight(18, 0.5); // turn right
//                    } else if (signal == 3) {
//                        encoderStrafeDriveInchesRight(-18, 0.5); // turn left
//                    }
//                    encoderStraightDrive(-36, 0.5);
//                }

                break;
            }

        } while (opModeIsActive());
        AutoTransitioner.transitionOnStop(this, "TeleOp");
    }
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        /**
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
        /**
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = Constants.minResultConfidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        /** if (labels != null) {
         tfod.loadModelFromFile(TFOD_MODEL_FILE, labels);
         }
         */
  //  }
//}
