/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class WebcamExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    OpenCvWebcam webcam;

    private DcMotor BackrightAsDcMotor = null;
    private DcMotor FrontrightAsDcMotor = null;
    private DcMotor FrontleftAsDcMotor = null;
    private DcMotor BackleftAsDcMotor = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;
    private DcMotor elevator = null;
    private DistanceSensor distanceSensor = null;
    private BNO055IMU imu;

    Orientation angles;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int left =1;
    int middle = 2;
    int right =3;

    AprilTagDetection tagOfInterest = null;

    int move=100;
    int go=100;
    @Override
    public void runOpMode()
    {
        BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
        BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
        FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
        FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
        leftServo = hardwareMap.crservo.get("left");
        rightServo = hardwareMap.crservo.get("right");
        elevator = hardwareMap.dcMotor.get("elevator");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        imu=hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        //webcam.setPipeline(new SamplePipeline());
        junctionPipeline junction = new junctionPipeline();
        blueConePipeline blueCone = new blueConePipeline();
        blueConeDistancePipline blueConeDistance = new blueConeDistancePipline();
        junctionDistancePipeline junctionDistance = new junctionDistancePipeline();


        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
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

                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

        telemetry.addLine("Waiting for start");
        telemetry.update();
        move=100;
        go=100;


        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        while (opModeIsActive())
        {
            in(0.7);
            elevatorUp(750, 1);
            forward(2500, 1);
            sleep(1500);
            backward(350,1);
            sleep(500);
            centerOn(0);
            sleep(500);
            elevatorUp(2000,1);

            BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            webcam.setPipeline(junctionDistance);

            while(opModeIsActive() && go!=1){
                BackrightAsDcMotor.setPower(0.2);
                BackleftAsDcMotor.setPower(0.2);
                FrontrightAsDcMotor.setPower(0.2);
                FrontleftAsDcMotor.setPower(0.2);
            }
            BackrightAsDcMotor.setPower(0);
            BackleftAsDcMotor.setPower(0);
            FrontrightAsDcMotor.setPower(0);
            FrontleftAsDcMotor.setPower(0);
            sleep(1000);

            forward(120,1);
            elevatorUp(1400,1);
            sleep(1000);
            forward(100,1);
            sleep(500);
            elevatorDown(1000,1);
            sleep(100);
            drop(1);
            sleep(500);
            elevatorDown(3400,1);
            backward(190,1);

            sleep(500);
            centerOn(0);
            sleep(500);
            turnright(837,0.2);
            sleep(2000);
            centerOnNinetyRight(90);
            sleep(500);
            forward(1600,1);
            sleep(500);
            centerOnNinetyRight(90);
            sleep(500);

            BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            webcam.setPipeline(blueConeDistance);

            while(opModeIsActive() && go!=1){
                BackrightAsDcMotor.setPower(0.2);
                BackleftAsDcMotor.setPower(0.2);
                FrontrightAsDcMotor.setPower(0.2);
                FrontleftAsDcMotor.setPower(0.2);
            }
            BackrightAsDcMotor.setPower(0);
            BackleftAsDcMotor.setPower(0);
            FrontrightAsDcMotor.setPower(0);
            FrontleftAsDcMotor.setPower(0);
            in(0.7);
            sleep(100);
            elevatorUp(1500,1);
            sleep(1000);
            forward(150,0.5);
            sleep(500);
            elevatorDown(600,1);
            sleep(1000);
            elevatorUp(600,1);
        /*
            sleep(1000);
            backward(400,1);
            sleep(1000);
            centerOnNinetyRight((90));
            sleep(1000);
            crabRight(70,1);
            sleep(1000);
            backward(1000,1);
            sleep(1000);
            turnleft(837,0.3);
            sleep(1000);
            centerOn(0);
            sleep(1000);
            BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            webcam.setPipeline(junctionDistance);

            while(opModeIsActive() && go!=1){
                BackrightAsDcMotor.setPower(0.2);
                BackleftAsDcMotor.setPower(0.2);
                FrontrightAsDcMotor.setPower(0.2);
                FrontleftAsDcMotor.setPower(0.2);
            }
            BackrightAsDcMotor.setPower(0);
            BackleftAsDcMotor.setPower(0);
            FrontrightAsDcMotor.setPower(0);
            FrontleftAsDcMotor.setPower(0);
            sleep(1000);
            forward(120,1);
            elevatorUp(1400,1);
            sleep(2000);
            forward(100,1);
            sleep(1000);
            elevatorDown(1000,1);
            sleep(100);
            drop(1);
            sleep(1000);
            elevatorDown(3400,1);
            backward(170,1);
            sleep(1000);
            centerOn(0);






            //Yo fyi right turn is "837" and 180 turn is "1675"
            //slower u turn the more accurate it is


            /*
             * Send some stats to the telemetry

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();


             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            /*in(0.5);
            elevatorUp(750, 1);
            forward(2500, 1);
            sleep(2000);
            backward(400,1);
            sleep(1000);
            crabLeft(670,1);
            sleep(1000);

            go=100;
            /*while(opModeIsActive() && go!=1){
                webcam.setPipeline(junction);
                while(move!=0 && opModeIsActive()){
                    if(move==-1){
                        turnleft(1,0.1);
                        continue;

                    }
                    else if(move==1){
                        turnright(1,0.1);
                        continue;

                    }
                    else if(move==0) {
                        telemetry.addLine("ANGLE ALIGNED. u really shouldn't be getting this message tho");
                        telemetry.update();
                        break;
                    }
                    else if(move==100){
                        continue;
                    }
                    else{
                        telemetry.addLine("BIG ERROR. THIS IS SO BAD");
                        telemetry.update();
                    }
                }

                webcam.setPipeline(junctionDistance);
                if(move==0 && go!=1){
                    forward(7,0.3);
                }
                else if(move==0 && go==2){
                    telemetry.addLine("WE IN POSITION");
                    telemetry.update();
                    break;
                }
                else{
                    telemetry.addLine("something bad happened");
                    telemetry.update();
                }
            }
            driveMotorStop();*/
            /*
            sleep(1000);
            forward(220,1);
            elevatorUp(4250,1);
            sleep(3000);
            elevatorDown(1000,1);
            sleep(500);
            drop(1);
            sleep(1000);
            backward(300,1);*/
            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(30000);
        }
    }
    private void centerOn(double angle){
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while(opModeIsActive() && (daAngle>(2) || daAngle<(-2))) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        daAngle=angles.firstAngle;
        if (daAngle > 2) {
            turnright(1, 0.3);
        } else if (daAngle < -2) {
            turnleft(1, 0.3);
        } else {
            telemetry.addLine("We got a problem");
            telemetry.update();
        }
    }
}
    private void centerOnNinetyRight(double angle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double daAngle = angles.firstAngle;
        while (opModeIsActive() && (daAngle > (-89) || daAngle < (-91))) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            daAngle = angles.firstAngle;
            if (daAngle > -89) {
                turnright(1, 0.3);
            } else if (daAngle < -91) {
                turnleft(1, 0.3);
            } else {
                telemetry.addLine("We got a problem");
                telemetry.update();
            }
        }
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

    class blueConeDistancePipline extends OpenCvPipeline
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
        public blueConeDistancePipline() {
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
            Scalar lowHSV = new Scalar(110,70,50); // lower blue
            Scalar highHSV = new Scalar(130,255,255);  // lenient higher bound HSV for yellow
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
            if (frameList.size() > 100) {
                frameList.remove(0);
            }

            scaledThresh.copyTo(input);
            Rect Rect=new Rect(
                    new Point(
                            input.width()/4f,
                            0),
                    new Point(
                            input.width()*(3f/4f),
                            input.height())
            );

            Imgproc.rectangle(input,
                    new Point(
                            input.width()*(3f/8f),
                            0),
                    new Point(
                            input.width()*(5f/8f),
                            input.height()),
                    new Scalar(255,0,0),5);

            Mat rect = input.submat(Rect);

            double rectValue = 100*Core.sumElems(rect).val[0]/ ((rect.width()/2)*(rect.height()))/255;

            telemetry.addLine(rectValue+"   "+go);
            if(rectValue>30) {
                go=1;
                telemetry.addLine("AT THE JUNCTION");
            }
            else if(rectValue<=30){
                go=2;
                telemetry.addLine("adjusting");
            }
            else{
                telemetry.addLine("THIS IS PRETTY BAD DUDE");
                telemetry.update();
            }
            telemetry.update();

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
            rect.release();

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

    class junctionDistancePipeline extends OpenCvPipeline
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
        public junctionDistancePipeline() {
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
            if (frameList.size() > 100) {
                frameList.remove(0);
            }

            scaledThresh.copyTo(input);
            Rect Rect=new Rect(
                    new Point(
                            input.width()/4f,
                            0),
                    new Point(
                            input.width()*(3f/4f),
                            input.height())
            );

            Imgproc.rectangle(input,
                    new Point(
                            input.width()*(3f/8f),
                            0),
                    new Point(
                            input.width()*(5f/8f),
                            input.height()),
                    new Scalar(255,0,0),5);

            Mat rect = input.submat(Rect);

            double rectValue = 100*Core.sumElems(rect).val[0]/ ((rect.width()/2)*(rect.height()))/255;

            telemetry.addLine(rectValue+"   "+go);
            if(rectValue>30) {
                go=1;
                telemetry.addLine("AT THE JUNCTION");
            }
            else if(rectValue<=30){
                go=2;
                telemetry.addLine("adjusting");
            }
            else{
                telemetry.addLine("THIS IS PRETTY BAD DUDE");
                telemetry.update();
            }
            telemetry.update();

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
            rect.release();

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

    class blueConePipeline extends OpenCvPipeline
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
        public blueConePipeline() {
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
            Scalar lowHSV = new Scalar(110,70,50); // lower blue
            Scalar highHSV = new Scalar(130,255,255); //higher blue (look i commented yay)
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

            telemetry.addLine(" "+leftValue+"              "+rightValue);
            telemetry.update();
            if(leftValue-rightValue>5 || rightValue-leftValue>5) {
                if (leftValue > rightValue) {
                    move = -1;
                    //telemetry.addLine("too right");
                    //telemetry.update();
                } else if (rightValue > leftValue) {
                    move = 1;
                    //telemetry.addLine("too left");
                    //telemetry.update();

                } else {
                    telemetry.addLine("SOMETHING IS VERY WRONG");
                    telemetry.update();
                }
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


    //movement functions
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
    private void drop(double power){
        rightServo.setPower(power);
        leftServo.setPower(power);
    }
    private void stopServos(){
        rightServo.setPower(0);
        leftServo.setPower(0);
    }
    private void in(double power){
        rightServo.setPower(-power);
        leftServo.setPower(-power);
    }
    private void crabLeft(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void crabRight(int distance, double power){
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turnright(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void turnleft(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontleftAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        BackrightAsDcMotor.setTargetPosition(distance);

        FrontrightAsDcMotor.setPower(power);
        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void backward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void forward(int distance, double power) {
        BackrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackleftAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        BackrightAsDcMotor.setTargetPosition(distance);
        BackleftAsDcMotor.setTargetPosition(distance);
        FrontrightAsDcMotor.setTargetPosition(distance);
        FrontleftAsDcMotor.setTargetPosition(distance);

        BackrightAsDcMotor.setPower(power);
        BackleftAsDcMotor.setPower(power);
        FrontrightAsDcMotor.setPower(power);
        FrontleftAsDcMotor.setPower(power);

        BackrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontrightAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontleftAsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void driveMotorStop(){
        BackrightAsDcMotor.setPower(0);
        BackleftAsDcMotor.setPower(0);
        FrontrightAsDcMotor.setPower(0);
        FrontleftAsDcMotor.setPower(0);
    }
}