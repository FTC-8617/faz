package bad_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.WebcamExample;
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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Kamikaze", preselectTeleOp = "MECHANUM")
public class Kamikaze extends LinearOpMode {

  private DcMotor BackrightAsDcMotor = null;
  private DcMotor FrontrightAsDcMotor = null;
  private DcMotor FrontleftAsDcMotor = null;
  private DcMotor BackleftAsDcMotor = null;
  private DcMotor encoderone = null;
  private DcMotor encodertwo = null;
  private CRServo leftServo = null;
  private CRServo rightServo = null;
  private DcMotor elevator = null;
  //private DcMotor tester = null;
  OpenCvWebcam webcam;

  private BNO055IMU imu;

  Orientation angles;

  int move=100;
  int go=100;
  boolean dark = false;
  int count=0;
  int aligncount=0;



  @Override
  public void runOpMode() {

    BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
    BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
    FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
    FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
    elevator = hardwareMap.get(DcMotor.class, "elevator");
    leftServo = hardwareMap.get(CRServo.class, "left");
    rightServo = hardwareMap.get(CRServo.class, "right");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    imu=hardwareMap.get(BNO055IMU.class,"imu");
    imu.initialize(parameters);

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    blueConePipeline blueCone = new Kamikaze.blueConePipeline();
    junctionPipeline junction = new Kamikaze.junctionPipeline();

    /*tester = hardwareMap.dcMotor.get("tester");
    tester.setDirection(DcMotorSimple.Direction.REVERSE);


    tester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    tester.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    tester.setTargetPosition(1000);


    tester.setPower(1);


    tester.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    telemetry.addLine(tester.getCurrentPosition()+ " ");
    telemetry.update();

    sleep(1000);

    tester.setDirection(DcMotorSimple.Direction.REVERSE);


    tester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    tester.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    tester.setTargetPosition(1000);


    tester.setPower(0.1);


    tester.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    telemetry.addLine(tester.getCurrentPosition()+ " ");
    telemetry.update();*/



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

    waitForStart();
    if (opModeIsActive()) {
      while (!isStopRequested()) {

        //move up
        crabLeft(1000,0.75);
        sleep(1300);
        in(0.01);
        center(0);
        centersomewhatprecise(0);
        forward(2190,.75);
        elevatorUp(1125,1);
        sleep(1550);
        center(0);
        crabRight(580,0.75);
        sleep(1000);
        //score high
        webcam.setPipeline(junction);
        count=0;
        while(move!=0 && opModeIsActive() && (count<35)){
          if(move==-1){
            turnleft(1,0.15);
            count++;
            sleep(20);
          }
          else if(move==1){
            turnright(1,0.15);
            count++;
            sleep(20);
          }
          else{
            telemetry.addLine("idk wut happened");
            telemetry.update();
          }
        }
        elevatorUp(1800,1);
        sleep(700);
        forward(150,0.4);
        sleep(1000);
        elevatorDown(500,1);
        sleep(300);
        drop(1);
        sleep(200);
        elevatorUp(700,1);
        sleep(500);
        drop(0);
        //go to stack
        backwards(120,0.75);
        sleep(600);
        turnright(300,0.75);
        sleep(300);
        turnright(520,0.75);
        elevatorDown(6000,1);
        sleep(750);
        centersomewhatprecise(-90);
        forward(1100,0.75);
        elevatorUp(40,1);
        sleep(820);
        centersomewhatprecise(-90);
        //crabLeft(145,0.75);
        //sleep(100);
        /*forward(860,0.75);
        elevatorUp(40,1);
        sleep(750);
        center(-90);
        crabLeft(50,0.75);
        sleep(100);
        forward(390,0.75);
        sleep(750);*/
        /*if(dark){
          elevatorDown(4000,1);
          sleep(1000);
          telemetry.addLine("DARK");
          telemetry.update();
        }*/
        webcam.setPipeline(blueCone);
        count=0;
        while(move!=0 && opModeIsActive()&&(count<50)){
          if(move==-1){
            turnleft(2,0.2);
            count++;
            sleep(20);
          }
          else if(move==1){
            turnright(2,0.2);
            count++;
            sleep(20);
          }
          else{
            telemetry.addLine("idk wut happened");
            telemetry.update();
          }
        }

        elevatorUp(670,1);
        sleep(700);
        forward(437,0.65);
        sleep(800);
        backwards(43,0.2);

        sleep(250);
        elevatorDown(230,1);
        in(.25);
        sleep(640);
        in(0);
        elevatorUp(1000,1);

        sleep(500);
        centersomewhatprecise(-90);
        backwards(500,0.75);
        sleep(350);
        turnright(820,0.75);
        sleep(1000);
        elevatorUp(500,1);
        sleep(1000);
        webcam.setPipeline(junction);
        count=0;
        while(move!=0 && opModeIsActive()&&(count<50)){
          if(move==-1){
            turnleft(1,0.15);
            count++;
            sleep(20);
          }
          else if(move==1){
            turnright(1,0.15);
            count++;
            sleep(20);
          }
          else{
            telemetry.addLine("idk wut happened");
            telemetry.update();
          }
        }

        forward(40, 0.6);
        elevatorDown(300,1);
        sleep(500);
        drop(1);
        sleep(300);
        drop(0);
        /*


        forward(50,0.75);
        in(0.5);
        elevatorDown(110,1);
        sleep(520);
        in(0);
        backwards(75,0.75);
        sleep(100);
        elevatorUp(1000,1);
        sleep(500);
        backwards(200,0.75);
        sleep(200);
        center(-90);
        //score on high
        crabRight(110,0.75);
        sleep(300);
        backwards(800,0.75);
        sleep(700);
        center(-90);
        turnleft(850,0.75);
        sleep(700);
        center(0);
        elevatorUp(1500,1);
        crabLeft(850,0.75);
        sleep(600);
        center(0);
        forward(153,1);
        sleep(400);
        elevatorDown(200,1);
        sleep(300);
        drop(1);










        */
        sleep(30000);
        webcam.stopStreaming();

      }
      telemetry.update();
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
      if(leftValue-rightValue>3|| rightValue-leftValue>3) {
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
      if(leftValue<14 && rightValue<14){
        dark=true;
      }
      else{
        dark=false;
      }
      if(leftValue-rightValue>4 || rightValue-leftValue>4) {
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

  private void centerbad(double angle){
    aligncount=0;
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while((daAngle>(angle+0.5) || daAngle<(angle-0.5)) && opModeIsActive()&&aligncount<50){
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if((daAngle-angle)>3 || (daAngle-angle)<-3){
        centerOn(3,4,0.3,angle);
      } else if((daAngle-angle)>1 || (daAngle-angle)<-1){
        centerOn(1,3,0.2,angle);
      } else if((daAngle-angle)>0.5 || (daAngle-angle)<-0.5){
        centerOn(0.5,2,0.2,angle);
      }
      else{
        telemetry.addLine("ALIGNED?     "+daAngle);
        telemetry.update();
      }
    }
  }
  private void center(double angle){
    aligncount=0;
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while((daAngle>(angle+0.1) || daAngle<(angle-0.1)) && opModeIsActive()&&(aligncount<50)){
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if((daAngle-angle)>3 || (daAngle-angle)<-3){
        centerOn(3,4,0.3,angle);
      } else if((daAngle-angle)>1 || (daAngle-angle)<-1){
        centerOn(1,3,0.2,angle);
      } else if((daAngle-angle)>0.5 || (daAngle-angle)<-0.5){
        centerOn(0.5,2,0.2,angle);
      } else if ((daAngle - angle) > 0.3 || (daAngle - angle) < -0.3){
        centerOn(0.3,1,0.2,angle);
      } else if ((daAngle - angle) > 0.1 || (daAngle - angle) < -0.1) {
        centerOn(0.1,1,0.1,angle);
      }
      else{
        telemetry.addLine("ALIGNED?     "+daAngle);
        telemetry.update();
      }
    }
  }
  private void centerSUPAPRECISE(double angle){
    aligncount=0;
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while((daAngle>(angle+0.05) || daAngle<(angle-0.05)) && opModeIsActive()&&(aligncount<50)){
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if((daAngle-angle)>3 || (daAngle-angle)<-3){
        centerOn(3,4,0.3,angle);
      } else if((daAngle-angle)>1 || (daAngle-angle)<-1){
        centerOn(1,3,0.2,angle);
      } else if((daAngle-angle)>0.5 || (daAngle-angle)<-0.5){
        centerOn(0.5,2,0.2,angle);
      } else if ((daAngle - angle) > 0.3 || (daAngle - angle) < -0.3){
        centerOn(0.3,1,0.2,angle);
      } else if ((daAngle - angle) > 0.1 || (daAngle - angle) < -0.1) {
        centerOn(0.1,1,0.1,angle);
      } else if((daAngle - angle) > 0.05 || (daAngle - angle) < -0.05){
        centerOn(0.05,1,0.1,angle);
      }
      else{
        telemetry.addLine("ALIGNED?     "+daAngle);
        telemetry.update();
      }
    }
  }
  private void centersomewhatprecise(double angle){
    aligncount=0;
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while((daAngle>(angle+0.05) || daAngle<(angle-0.05)) && opModeIsActive()&&(aligncount<50)){
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if((daAngle-angle)>3 || (daAngle-angle)<-3){
        centerOn(3,4,0.3,angle);
      } else if((daAngle-angle)>1 || (daAngle-angle)<-1){
        centerOn(1,3,0.2,angle);
      } else if((daAngle-angle)>0.5 || (daAngle-angle)<-0.5){
        centerOn(0.5,2,0.2,angle);
      } else if ((daAngle - angle) > 0.3 || (daAngle - angle) < -0.3){
        centerOn(0.3,1,0.2,angle);
      } else if ((daAngle - angle) > 0.1 || (daAngle - angle) < -0.1) {
        centerOn(0.1,1,0.1,angle);
      } else if((daAngle - angle) > 0.07 || (daAngle - angle) < -0.07){
        centerOn(0.07,1,0.1,angle);
      }
      else{
        telemetry.addLine("ALIGNED?     "+daAngle);
        telemetry.update();
      }
    }
  }
  private void centerOn(double accuracy, int distance, double power, double angle){
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle=angles.firstAngle;
    while(opModeIsActive() && (daAngle>(accuracy+angle) || daAngle<(angle-accuracy))) {
      aligncount++;
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle=angles.firstAngle;
      if (daAngle >= (accuracy+angle)) {
        turnright(distance, power);
        sleep(20);
      } else if (daAngle <= (angle-accuracy)) {
        turnleft(distance, power);
        sleep(20);
      } else {
        telemetry.addLine("We got a problem");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine("angle: "+angles.firstAngle);
        telemetry.update();
      }
    }
  }
  private void centerOnNinetyRight(double accuracy, int distance, double power) {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double daAngle = angles.firstAngle;
    while (opModeIsActive() && (daAngle > (-90+accuracy) || daAngle < (-90-accuracy))) {
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      daAngle = angles.firstAngle;
      if (daAngle >= -90+accuracy) {
        turnright(distance, power);
        sleep(20);
      } else if (daAngle <= (-90-accuracy)) {
        turnleft(distance, power);
        sleep(20);
      } else {
        telemetry.addLine("We got a problem");
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine("angle: "+angles.firstAngle);
        telemetry.update();
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

  private void backwards(int distance, double power) {
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
    FrontrightAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

  private void forward(int distance, double power) {
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

  private void drop(double power){
    leftServo.setDirection(DcMotorSimple.Direction.FORWARD);
    rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    leftServo.setPower(power);
    rightServo.setPower(power);
  }
  private void in(double power){
    rightServo.setDirection(DcMotorSimple.Direction.FORWARD);
    leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
    leftServo.setPower(power);
    rightServo.setPower(power);
  }
}
