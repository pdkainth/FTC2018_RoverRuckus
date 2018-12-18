package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing VuMark teleOP")

public class Test_VuMark_TeleOp extends OpMode {

  public enum TestMode {
    ACQUIRE,
    GOTO_HOME
  }

  public enum Rotate {
    IDLE {
      public Rotate update(Test_VuMark_TeleOp teleOp) { return IDLE; }
    },
    START {
      public Rotate update(Test_VuMark_TeleOp teleOp) {
        float angleDelta = Gyro.convertAngle180(teleOp.robotPos.angle - teleOp.desiredAngle);
        teleOp.mecanum.drive(0.0, 0.0f,Math.signum(angleDelta) * 0.5, teleOp.telemetry);
        return TURN;
      }
    },
    TURN {
      public Rotate update(Test_VuMark_TeleOp teleOp) {
        teleOp.gyro.update(teleOp.telemetry);
        teleOp.robotPos.angle = teleOp.gyro.getHeading();
        float angleDelta = Gyro.convertAngle180(teleOp.robotPos.angle - teleOp.desiredAngle);
        if (Math.abs(angleDelta) < 1.0f) {
          teleOp.mecanum.stop();
          return IDLE;
        } else {
          teleOp.mecanum.drive(0.0, 0.0f,Math.signum(angleDelta) * 0.5, teleOp.telemetry);
          return TURN;
        }
      }
    };

    public abstract Rotate update(Test_VuMark_TeleOp teleOp);
  }

  public enum GoToHomeState {
    IDLE {
      public GoToHomeState update(Test_VuMark_TeleOp teleOp) { return IDLE; }
    },
    START {
      public GoToHomeState update(Test_VuMark_TeleOp teleOp) {
        teleOp.desiredAngle = teleOp.robotPos.slopeTo(teleOp.homePos);
        teleOp.driveDistance = teleOp.robotPos.distance(teleOp.homePos);

        teleOp.rotState = Rotate.START;
        return ORIENT_TO_HOME;
      }
    },
    ORIENT_TO_HOME {
      public GoToHomeState update(Test_VuMark_TeleOp teleOp) {
        if (teleOp.rotState != Rotate.IDLE) {
          teleOp.rotState = teleOp.rotState.update(teleOp);
          return ORIENT_TO_HOME;
        } else {
          // reset encoder for wheels
          // calculate encoder for distance and set wheels to motion
          teleOp.mecanum.setTargetDrive(teleOp.driveDistance);
          teleOp.mecanum.drive(0.5, 0.0, 0.0, teleOp.telemetry);
          return MOVE_TO_HOME;
        }
      }
    },
    MOVE_TO_HOME {
      public GoToHomeState update(Test_VuMark_TeleOp teleOp) {

        // watch encoder count, with in range, stop wheels
        if (teleOp.mecanum.isTargetDriveDone()) {
          teleOp.desiredAngle = teleOp.homePos.angle;
          teleOp.rotState = Rotate.START;
          return FINAL_ORIENT;
        } else {
          return MOVE_TO_HOME;
        }
      }
    },
    FINAL_ORIENT {
      public GoToHomeState update(Test_VuMark_TeleOp teleOp) {
        if (teleOp.rotState != Rotate.IDLE) {
          teleOp.rotState = teleOp.rotState.update(teleOp);
          return FINAL_ORIENT;
        } else {
          return IDLE;
        }
      }
    };

    public abstract GoToHomeState update(Test_VuMark_TeleOp teleOp);
  }

  private ElapsedTime runtime = new ElapsedTime();
  private VuMark_Nav nav = new VuMark_Nav();
  private Gyro gyro = new Gyro();
  private Wheels mecanum = new Wheels();
  private PhoneTilt phoneTilt = new PhoneTilt();

  boolean headingOffsetUpdate = true;
  Position robotPos = new Position();

  Position homePos  = new Position(-60.0f, 0.0f, 90.0f);
  GoToHomeState homeState = GoToHomeState.IDLE;
  Rotate        rotState  = Rotate.IDLE;
  public float desiredAngle;
  public float driveDistance;

  private TestMode testMode = TestMode.ACQUIRE;

  @Override
  public void init(){
    nav.init(hardwareMap);
    gyro.init(hardwareMap);
    mecanum.init(hardwareMap);
    phoneTilt.init(hardwareMap);
  }

  @Override
  public void init_loop(){}

  @Override
  public void start(){
    runtime.reset();
    nav.start();
    phoneTilt.setPhoneTilt(PhoneTilt.PHONE_TILT_DEG_VUFORIA);
  }

  @Override
  public void loop(){

    if (testMode == TestMode.ACQUIRE) {
      nav.scan(telemetry);
      gyro.update(telemetry);

      if (nav.isTargetValid()) {
        if (headingOffsetUpdate) {
          headingOffsetUpdate = false;
          gyro.updateHeadingOffset(nav.getHeading());
        }
        robotPos.setPos(nav.getPosX(), nav.getPosY(), gyro.getHeading());

        if (gamepad1.x == true) {
          testMode = TestMode.GOTO_HOME;
          homeState = GoToHomeState.START;
        }
      } else {
        headingOffsetUpdate = true;
      }

      updateWheels();
      phoneTilt.update(telemetry);
    } else {
      if (homeState != GoToHomeState.IDLE) {
        homeState = homeState.update(this);
      } else {
        testMode = TestMode.ACQUIRE;
      }
    }

    telemetry.addData("TestVuMark", "mode %s home %s rot %s target angle %.1f dist %.1f",
        testMode, homeState, rotState, desiredAngle, driveDistance);
    telemetry.addData("TestVuMark1", "Cur %s home %s slope %.1f distance %.1f",
        robotPos, homePos, robotPos.slopeTo(homePos), robotPos.distance(homePos));
  }

  @Override
  public void stop(){
    nav.stop();
    mecanum.stop();
    phoneTilt.stop();
  }

  public void updateWheels(){

    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if(gamepad1.dpad_left == true){
      strafe = -0.6;
    } else if(gamepad1.dpad_right == true){
      strafe = 0.6;
    }

    double rotate = gamepad1.right_stick_x;

    mecanum.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("Wheel","Wheel power: D: %.2f S: %.2f R: %.2f", drive, strafe, rotate);
  }
}
