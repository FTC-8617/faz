package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="BackupMECHANUM", group="TeleOp")
public class BackupMECHANUM extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;
    private final double driveAdjuster = 1;
    double  elevPower = 0;
    OpenCvWebcam webcam;
    int move=100;

    public enum LiftState{
        bottom,intake,outtake,starttransition, init,start,high
    };
    LiftState liftState = LiftState.init;

    
    @Override
    public void runOpMode()  throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        junctionPipeline junction = new junctionPipeline();
        webcam.setPipeline(junction);



        
        
        frontLeft = hardwareMap.dcMotor.get("front left");
        backLeft = hardwareMap.dcMotor.get("back left");
        frontRight = hardwareMap.dcMotor.get("front right");
        backRight = hardwareMap.dcMotor.get("back right");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        rightServo.setDirection(CRServo.Direction.REVERSE);
        leftServo.setDirection(CRServo.Direction.FORWARD);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */

                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 5);

            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }

        });

        waitForStart();






        elevator.setPower(1);


        while (opModeIsActive()) {
            switch(liftState){
                case init:
                    elevatorUp(300,1);
                    liftState=LiftState.starttransition;
                    break;
                case start:
                    if(gamepad2.dpad_up){
                        elevatorUp(2600,1);
                        liftState= LiftState.high;
                    }
                    else if(gamepad2.dpad_down){
                        elevatorDown(300,1);
                        liftState=LiftState.bottom;
                    }
                    else{
                        telemetry.addLine("ummm");
                    }
                    break;
                case bottom:
                    if (Math.abs(elevator.getCurrentPosition()-0)<10){
                        if(gamepad2.dpad_up){
                            elevatorUp(300,1);
                            liftState=LiftState.starttransition;
                        }
                    }
                    else{
                        telemetry.addLine(elevator.getCurrentPosition()+" ");
                    }
                    break;
                case high:
                    if(Math.abs(elevator.getCurrentPosition()-2600)<10) {
                        if(gamepad2.dpad_down){
                            elevatorDown(2600,1);
                            liftState= LiftState.starttransition;
                        }

                    }
                    else{
                        telemetry.addLine("WHYYY");
                    }
                    break;
                case starttransition:
                    if(Math.abs(elevator.getCurrentPosition()-300)<10){
                        liftState =LiftState.start;
                    }
                    break;
                default:
                    liftState = LiftState.start;


            }

            telemetry.addLine(" "+liftState);

            if(gamepad2.circle && liftState !=LiftState.start){
                liftState=LiftState.start;
            }

            if(move==0){
                gamepad1.rumble(50);
            }
            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;

            if(gamepad1.left_stick_y > 0.15 || gamepad1.left_stick_y < -0.15 || gamepad1.left_stick_x > 0.15 || gamepad1.left_stick_x < -0.15) {
                frontLeft.setPower(-v3);
                frontRight.setPower(-v4);
                backLeft.setPower(-v1);
                backRight.setPower(-v3);
            }

            else if(gamepad1.left_bumper) {
                frontLeft.setPower(-0.75);
                backLeft.setPower(-0.75);
                frontRight.setPower(0.75);
                backRight.setPower(0.75);
            }

            else if(gamepad1.right_bumper) {
                frontLeft.setPower(0.75);
                backLeft.setPower(0.75);
                frontRight.setPower(-0.75);
                backRight.setPower(-0.75);
            }
            else if(gamepad1.right_trigger > 0.05) {
                double power = 0.5*gamepad1.right_trigger;
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);
            }
            else if(gamepad1.left_trigger > 0.05) {
                double power = 0.5*gamepad1.left_trigger;
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);
            }
            else {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            // Spins wheels out
            if(gamepad2.right_trigger > 0.05){
                double servoPower = 0.3*gamepad2.right_trigger;
                rightServo.setPower(servoPower);
                leftServo.setPower(servoPower);
            }
            // Spins wheels in
            else if(gamepad2.left_trigger > 0.05){
                double servoPower = 0.33*gamepad2.left_trigger;
                rightServo.setPower(-servoPower);
                leftServo.setPower(-servoPower);
            }
            else{
                rightServo.setPower(0);
                leftServo.setPower(0);
            }





        }
    }
    private void elevatorUp(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void elevatorDown(int distance, double power){
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        elevator.setTargetPosition(distance);

        elevator.setPower(power);

        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    class junctionPipeline extends OpenCvPipeline
    {
        /*
         * An example image processing pipeline to be run upon receipt of each frame from the camera.
         * Note that the processFrame() method is called serially from the frame worker thread -
         * that is, a new camera frame will not come in while you're still processing a previous one.
         * In other words, the processFrame() method will never be called multiple times simultaneously.
         *
         * However, the rendering of your processed image to the viewport is done in parallel to the
         * frame worker thread. That is, the amount of time it takes to render the image to the
         * viewport does NOT impact the amount of frames per second that your pipeline can process.
         *
         * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
         * frame worker thread. This should not be a problem in the vast majority of cases. However,
         * if you're doing something weird where you do need it synchronized with your OpMode thread,
         * then you will need to account for that accordingly.
         */
        boolean viewportPaused;


        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        ArrayList<double[]> frameList;
        public junctionPipeline() {
            frameList = new ArrayList<>();
        }

        public double strictLowS = 140;
        public double strictHighS = 255;

        @Override
        public Mat processFrame(Mat input) {
            //Mat mat = new Mat();

            //mat turns into HSV value
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            if (input.empty()) {
                telemetry.addLine("nothin");
                telemetry.update();
                return input;
            }

            // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
            Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
            Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow
            Mat thresh = new Mat();

            // Get a black and white image of yellow objects
            Core.inRange(input, lowHSV, highHSV, thresh);

            Mat masked = new Mat();
            //color the white portion of thresh in with HSV from mat
            //output into masked
            Core.bitwise_and(input, input, masked, thresh);
            //calculate average HSV values of the white thresh values
            Scalar average = Core.mean(masked, thresh);

            Mat scaledMask = new Mat();
            //scale the average saturation to 150
            masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


            Mat scaledThresh = new Mat();
            //you probably want to tune this
            Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
            Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
            //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
            Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

            Mat finalMask = new Mat();
            //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
            Core.bitwise_and(input, input, finalMask, scaledThresh);

            Mat edges = new Mat();
            //detect edges(only useful for showing result)(you can delete)
            Imgproc.Canny(scaledThresh, edges, 100, 200);

            //contours, apply post processing to information
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            //find contours, input scaledThresh because it has hard edges
            Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
            if (frameList.size() > 5) {
                frameList.remove(0);
            }

            scaledThresh.copyTo(input);
            Rect leftRect=new Rect(
                    new Point(
                            0,
                            0),
                    new Point(
                            input.width()/2,
                            (1)*(input.height()))
            );
            Rect rightRect=new Rect(
                    new Point(
                            input.width()/(2),
                            0),
                    new Point(
                            input.width()*(1),
                            (1)*(input.height()))
            );

            Imgproc.rectangle(input,
                    new Point(
                            0,
                            0),
                    new Point(
                            input.width()/2,
                            (1)*(input.height())),
                    new Scalar(255,0,0),5);
            Imgproc.rectangle(input,
                    new Point(
                            input.width()/(2),
                            0),
                    new Point(
                            input.width()*(1),
                            (1)*(input.height())),
                    new Scalar(255,0,0),5);

            Mat left = input.submat(leftRect);
            Mat right = input.submat(rightRect);

            double leftValue = 100*Core.sumElems(left).val[0]/ ((left.width()*left.height())/2)/255;
            double rightValue = 100*Core.sumElems(right).val[0]/ ((right.width()*right.height())/2)/255;

            telemetry.addLine(" "+leftValue+"              "+rightValue+"                "+move);
            telemetry.update();
            if(leftValue-rightValue>2|| rightValue-leftValue>2) {
                if (leftValue > rightValue) {
                    move = -1;
                    telemetry.addLine("too right");
                    telemetry.update();
                } else if (rightValue > leftValue) {
                    move = 1;
                    telemetry.addLine("too left");
                    telemetry.update();

                } else {
                    telemetry.addLine("SOMETHING IS VERY WRONG");
                    telemetry.update();
                }
            }
            else if((int)rightValue==7 && (int)leftValue==7){
                move=100;
            }
            else {
                move=0;
            }

            //Core.extractChannel(leftSide,leftSide,2);
            //Core.extractChannel(rightSide,rightSide,2);

            //Scalar leftAvg=Core.mean(leftSide).val(0);
            //Scalar rightAvg=Core.mean(rightSide).val(0);




            //release all the data*/
            //input.release();

            scaledThresh.release();
            scaledMask.release();
            //mat.release();
            masked.release();
            edges.release();
            thresh.release();
            finalMask.release();
            hierarchy.release();
            left.release();
            right.release();

            //change the return to whatever mat you want
            //for example, if I want to look at the lenient thresh:
            // return thresh;
            // note that you must not do thresh.release() if you want to return thresh
            // you also need to release the input if you return thresh(release as much as possible)

            //double leftTotal= Core.sumElems(left);*/

            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
    private void turnright(int distance, double power) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);

        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(power);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turnleft(int distance, double power) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(distance);
        frontRight.setTargetPosition(distance);
        backRight.setTargetPosition(distance);

        frontRight.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        frontLeft.setPower(power);

        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
