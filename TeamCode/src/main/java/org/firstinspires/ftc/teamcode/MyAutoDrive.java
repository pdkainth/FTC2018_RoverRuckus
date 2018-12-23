package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyAutoDrive extends OpMode {

  public enum AllianceColor {UNKNOWN, RED, BLUE}
  public enum AllianceSide  {UNKNOWN, CRATER, DEPOT}

  private ElapsedTime runtime = new ElapsedTime();

  private Wheels mecanum = new Wheels();
  private Intake intake = new Intake();
  private MineralDeliver minDel = new MineralDeliver();
  private Lift lift = new Lift();
  private VuMark_Nav nav = new VuMark_Nav();
  private Gyro gyro = new Gyro();
  private PhoneTilt phoneTilt = new PhoneTilt();
  private MineralDetect mineralDetect = new MineralDetect();

  Position robotPos = new Position();
  Position targetPos  = new Position();

  Rotate rotState   = Rotate.IDLE;
  Drive  driveState = Drive.IDLE;
  GoToTargetPos gotoTargetState = GoToTargetPos.IDLE;
  public float targetAngle;
  public float driveDistance;

  private MineralDetect.Position goldMineralPos = MineralDetect.Position.UNKNOWN;

  public enum AutoState {
    IDLE {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return IDLE;
      }
    },
    START {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        return GO_DOWN;
      }
    },
    GO_DOWN{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.lift.resetEncoder();
        autoDrive.lift.liftToPosition(Lift.LIFT_ENC_POS_HANG);
        return WAIT_GO_DOWN_FINISH;
      }
    },
    WAIT_GO_DOWN_FINISH{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if(autoDrive.lift.isBusy()){
          return WAIT_GO_DOWN_FINISH;
        } else {
          return SET_GET_OUT_OF_HOOK;
        }
      }
    },
    SET_GET_OUT_OF_HOOK{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        autoDrive.driveDistance = 3.0f;
        autoDrive.driveState = Drive.START;
        return GET_OUT_OF_HOOK;
      }
    },
    GET_OUT_OF_HOOK{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        if (autoDrive.driveState != Drive.IDLE){
          autoDrive.driveState = autoDrive.driveState.update(autoDrive);
          return GET_OUT_OF_HOOK;
        } else {
          autoDrive.lift.liftToPosition(1000);
          return PREP_TFO_SCAN;
        }
      }
    },

    PREP_TFO_SCAN {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.phoneTilt.setPhoneTilt(PhoneTilt.PHONE_TILT_DEG_TENSOR_FLOW);
        autoDrive.mineralDetect.start(autoDrive.telemetry);
        return TFO_SCAN;
      }
    },

    TFO_SCAN {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.goldMineralPos = autoDrive.mineralDetect.detect(autoDrive.telemetry);
        if (autoDrive.goldMineralPos == MineralDetect.Position.UNKNOWN) {
          autoDrive.goldMineralPos = autoDrive.mineralDetect.detectUsingLeft2(autoDrive.telemetry);
        }

        if (autoDrive.goldMineralPos == MineralDetect.Position.UNKNOWN) {
          return TFO_SCAN;
        } else {
          return SET_MOVE_AWAY_FROM_LANDER;
        }
      }
    },

    SET_MOVE_AWAY_FROM_LANDER{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        autoDrive.targetPos.setPos(autoDrive.robotPos);
        autoDrive.targetPos.angle = 160;
        autoDrive.gotoTargetState = GoToTargetPos.START;
        return MOVE_AWAY_FROM_LANDER;
      }
    },
    MOVE_AWAY_FROM_LANDER{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        if(autoDrive.gotoTargetState != GoToTargetPos.IDLE){
          autoDrive.gotoTargetState = autoDrive.gotoTargetState.update(autoDrive);
          return MOVE_AWAY_FROM_LANDER;
        } else {
          return PREP_VU_MARK_READ;
        }
      }
    },
    PREP_VU_MARK_READ {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        autoDrive.phoneTilt.setPhoneTilt(PhoneTilt.PHONE_TILT_DEG_VUFORIA);
        autoDrive.nav.start();
        return GET_ROBOT_POSITION;
      }
    },
    GET_ROBOT_POSITION {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        autoDrive.nav.scan(autoDrive.telemetry);
        if (autoDrive.nav.isTargetValid() == true) {
          autoDrive.gyro.updateHeadingOffset(autoDrive.nav.getHeading());
          autoDrive.robotPos.setPos(autoDrive.nav.getPosX(), autoDrive.nav.getPosY(), autoDrive.gyro.getHeading());
          return IDLE;
        } else{
          return GET_ROBOT_POSITION;
        }
      }
    }


    ;

    public abstract AutoState update(MyAutoDrive autoDrive, Telemetry telemetry);
  } // end of AutoState enum

  public enum Rotate {
    IDLE {
      public Rotate update(MyAutoDrive op) { return IDLE; }
    },
    START {
      public Rotate update(MyAutoDrive op) {
        float angleDelta = Gyro.convertAngle180(op.robotPos.angle - op.targetAngle);
        if (angleDelta < 1.0f) {
          return IDLE;
        } else {
          op.mecanum.drive(0.0, 0.0f, Math.signum(angleDelta) * 0.5, op.telemetry);
          return TURN;
        }
      }
    },
    TURN {
      public Rotate update(MyAutoDrive op) {
        op.gyro.update(op.telemetry);
        op.robotPos.angle = op.gyro.getHeading();
        float angleDelta = Gyro.convertAngle180(op.robotPos.angle - op.targetAngle);
        if (Math.abs(angleDelta) < 1.0f) {
          op.mecanum.stop();
          return IDLE;
        } else {
          op.mecanum.drive(0.0, 0.0f,Math.signum(angleDelta) * 0.5, op.telemetry);
          return TURN;
        }
      }
    };

    public abstract Rotate update(MyAutoDrive op);
  }

  public enum Drive {
    IDLE {
      public Drive update(MyAutoDrive op) { return IDLE; }
    },
    START {
      public Drive update(MyAutoDrive op) {
        if (op.driveDistance == 0.0f) {
          return IDLE;
        } else {
          op.mecanum.setTargetDrive(op.driveDistance);
          op.mecanum.drive(0.5, 0.0, 0.0, op.telemetry);
          return MOVE;
        }
      }
    },
    MOVE {
      public Drive update(MyAutoDrive op) {
        // watch encoder count, with in range, stop wheels
        if (op.mecanum.isTargetDriveDone()) {
          return IDLE;
        } else {
          return MOVE;
        }
      }
    };

    public abstract Drive update(MyAutoDrive op);
  }

  public enum GoToTargetPos {
    IDLE {
      public GoToTargetPos update(MyAutoDrive op) { return IDLE; }
    },
    START {
      public GoToTargetPos update(MyAutoDrive op) {
        op.targetAngle = op.robotPos.slopeTo(op.targetPos);
        op.driveDistance = op.robotPos.distance(op.targetPos);

        op.rotState = Rotate.START;
        return ORIENT_TO_HOME;
      }
    },
    ORIENT_TO_HOME {
      public GoToTargetPos update(MyAutoDrive op) {
        if (op.rotState != Rotate.IDLE) {
          op.rotState = op.rotState.update(op);
          return ORIENT_TO_HOME;
        } else {
          op.driveState = Drive.START;
          return MOVE_TO_HOME;
        }
      }
    },
    MOVE_TO_HOME {
      public GoToTargetPos update(MyAutoDrive op) {
        if (op.driveState != Drive.IDLE){
          op.driveState = op.driveState.update(op);
          return  MOVE_TO_HOME;
        } else {
          op.targetAngle = op.targetPos.angle;
          op.rotState = Rotate.START;
          return FINAL_ORIENT;
        }
      }
    },
    FINAL_ORIENT {
      public GoToTargetPos update(MyAutoDrive op) {
        if (op.rotState != Rotate.IDLE) {
          op.rotState = op.rotState.update(op);
          return FINAL_ORIENT;
        } else {
          return IDLE;
        }
      }
    };

    public abstract GoToTargetPos update(MyAutoDrive op);
  }

  private AllianceColor allianceColor = AllianceColor.UNKNOWN;
  private AllianceSide  allianceSide  = AllianceSide.UNKNOWN;

  private AutoState autoState;

  public void setAlliance(AllianceColor color, AllianceSide side) {
    allianceColor = color;
    allianceSide = side;
  }

  public void init() {
    telemetry.addData("AutoStatus", "Initializing");

    mecanum.init(hardwareMap);
    intake.init(hardwareMap);
    minDel.init(hardwareMap);
    lift.init(hardwareMap);
    nav.init(hardwareMap);
    gyro.init(hardwareMap);
    phoneTilt.init(hardwareMap);
    mineralDetect.init(hardwareMap, nav.getVuforia());

    autoState = AutoState.IDLE;

    telemetry.addData("AutoStatus", "Initialized A %s %s", allianceColor, allianceSide);
  }

  public void init_loop() {
    updateWheels();
    updateIntake();
    updateMineral();
    updateLift();
  }

  public void start() {
    runtime.reset();
    gyro.updateHeadingOffset(-135.0f);
    robotPos.setPos(-18.0f, 14.0f, -135.0f);
    autoState = AutoState.START;
    phoneTilt.setPhoneTilt(PhoneTilt.PHONE_TILT_DEG_VUFORIA);
    telemetry.addData("AutoStatus", "Started A %s %s states auto %s",
        allianceColor, allianceSide, autoState);
 }

  public void loop() {
    autoState = autoState.update(this, telemetry);

    telemetry.addData("AutoStatus", "runtime: %s A %s %s",
      runtime.toString(), allianceColor, allianceSide);

//    intake.update(telemetry);
//    minDel.update(telemetry);
//    lift.update(telemetry);
      gyro.update(telemetry);
      phoneTilt.update(telemetry);

    telemetry.addData("AutoState", "auto %s rot %s vuMark %s GoldPos %s",
      autoState, rotState, nav.getVisibleTarget(), goldMineralPos);
    telemetry.addData("AutoDrive", "POS target %s cur %s target angle %.1f dist %.1f",
      targetPos, robotPos, targetAngle, driveDistance);
  }

  public void stop() {
    mecanum.stop();
    intake.stop();
    minDel.stop();
    lift.stop();
    nav.stop();
    phoneTilt.stop();
  }

  public void updateWheels(){

    double drive = -gamepad1.left_stick_y;
    double strafe = 0.0;

    if(gamepad1.dpad_left == true){
      strafe = -1.0;
    } else if(gamepad1.dpad_right == true){
      strafe = 1.0;
    }

    double rotate = gamepad1.right_stick_x;

    mecanum.drive(drive, strafe, rotate, telemetry);
    telemetry.addData("Wheel","Wheel power: D: %.2f S: %.2f R: %.2f", drive, strafe, rotate);
  }

  public void updateIntake(){

    boolean on = gamepad1.right_bumper;
    boolean off = gamepad1.left_bumper;

    if(on){
      intake.turnOn();
    } else if(off){
      intake.turnOff();
    }

    boolean up = gamepad1.b;
    boolean down = gamepad1.x;

    if(up){
      intake.goUp();
    } else if(down){
      intake.goDown();
    }

    intake.update(telemetry);
  }

  public void updateMineral(){
    if (gamepad2.dpad_down == true) {
      minDel.resetEncoder();
    }

    double mineralMotorPower = -gamepad2.left_stick_y;
    minDel.upDn(mineralMotorPower);
    if(gamepad2.left_bumper){
      minDel.collect();
    } else if(gamepad2.right_bumper){
      minDel.dump();
    }
    minDel.update(telemetry);
  }

  public void updateLift(){
    if (gamepad2.dpad_up == true) {
      lift.resetEncoder();
    }

    double liftPower = -gamepad2.right_stick_y;
    lift.lift(liftPower);
    lift.update(telemetry);
  }
}