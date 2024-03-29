package bad_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "FazRight(old)", preselectTeleOp = "bad_code.FazRight")
public class FazRight extends LinearOpMode {

  private DcMotor BackrightAsDcMotor = null;
  private DcMotor FrontrightAsDcMotor = null;
  private DcMotor FrontleftAsDcMotor = null;
  private DcMotor BackleftAsDcMotor = null;
  private CRServo leftServo = null;
  private CRServo rightServo = null;
  private DcMotor elevator = null;
  
  @Override
  public void runOpMode() {
    
    BackrightAsDcMotor = hardwareMap.get(DcMotor.class, "back right");
    BackleftAsDcMotor = hardwareMap.get(DcMotor.class, "back left");
    FrontrightAsDcMotor = hardwareMap.get(DcMotor.class, "front right");
    FrontleftAsDcMotor = hardwareMap.get(DcMotor.class, "front left");
    leftServo = hardwareMap.crservo.get("left");
    rightServo = hardwareMap.crservo.get("right");
    elevator = hardwareMap.dcMotor.get("elevator");

    waitForStart();
    if (opModeIsActive()) {
      while (!isStopRequested()) {
        
        crabLeft(1850, 0.3);
        sleep(3000);
        forward(1200, 0.3);
        sleep(2000);
        elevatorUp(4000, 0.75);
        sleep(3000);
        forward(100, 0.25);
        forward(175, 0.15);
        drop(1.0);
        sleep(2000);
        forward(-250,0.25);
        sleep(2000);
        elevatorDown(4000, 0.50);
        sleep(4000);
        crabRight(1700, 0.3);
        sleep(3000);
        requestOpModeStop();
      }
      telemetry.update();
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
  
  private void drop(double power){
    rightServo.setPower(power);
    leftServo.setPower(power);
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

  /**
   * Describe this function...
   */
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

  /**
   * Describe this function...
   */
  private void backward(int distance, double power) {
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
}
