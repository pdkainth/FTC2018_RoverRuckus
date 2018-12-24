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

  protected AutoRoute autoRoute = new AutoRoute();

  Position robotPos = new Position();
  Position targetPos  = new Position();

  Rotate rotState   = Rotate.IDLE;
  Drive  driveState = Drive.IDLE;
  GoToTargetPos gotoTargetState = GoToTargetPos.IDLE;
  public float targetAngle;
  public float driveDistance;

  private double rotPower = 0.0;
  private double drivePower = 0.0;
  private float angleDelta = 0.0f;

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
        autoDrive.targetAngle = autoDrive.autoRoute.vuforiaAngle;
        autoDrive.rotState = Rotate.START;
        return MOVE_AWAY_FROM_LANDER;
      }
    },
    MOVE_AWAY_FROM_LANDER{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry){
        if(autoDrive.rotState != Rotate.IDLE){
          autoDrive.rotState = autoDrive.rotState.update(autoDrive);
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
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        autoDrive.nav.scan(autoDrive.telemetry);
        if (autoDrive.nav.isTargetValid() == true) {
          autoDrive.gyro.updateHeadingOffset(autoDrive.nav.getHeading());
          autoDrive.robotPos.setPos(autoDrive.nav.getPosX(), autoDrive.nav.getPosY(), autoDrive.gyro.getHeading());
          //return IDLE;
          return SET_MINERAL_POS;

        } else {
          return GET_ROBOT_POSITION;
        }
      }
    },

    SET_MINERAL_POS {
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int mineralPosIndex = autoDrive.goldMineralPos.indexOf();
        autoDrive.targetPos.setPos(autoDrive.autoRoute.mineralPos[mineralPosIndex]);
        autoDrive.gotoTargetState = GoToTargetPos.START;
        return GO_TO_MINERAL_POS;
      }
    },
    GO_TO_MINERAL_POS{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if(autoDrive.gotoTargetState != GoToTargetPos.IDLE){
          autoDrive.gotoTargetState = autoDrive.gotoTargetState.update(autoDrive);
          return GO_TO_MINERAL_POS;
        } else {
          int mineralPosIndex = autoDrive.goldMineralPos.indexOf();
          autoDrive.robotPos.setPos(autoDrive.autoRoute.mineralPos[mineralPosIndex]);
          return SET_MINERAL_KNOCKOUT;
        }
      }
    },
    SET_MINERAL_KNOCKOUT{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int mineralPosIndex = autoDrive.goldMineralPos.indexOf();
        autoDrive.driveDistance = autoDrive.autoRoute.mineralMoveDistance[mineralPosIndex];
        autoDrive.driveState = Drive.START;
        return WAIT_FOR_KNOCKOUT;
      }
    },
    WAIT_FOR_KNOCKOUT{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.driveState != Drive.IDLE){
          autoDrive.driveState = autoDrive.driveState.update(autoDrive);
          return WAIT_FOR_KNOCKOUT;
        } else {
          return SET_MINERAL_KNOCKOUT_MOVE_BACK;
        }
      }
    },
    SET_MINERAL_KNOCKOUT_MOVE_BACK{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        int mineralPosIndex = autoDrive.goldMineralPos.indexOf();
        autoDrive.driveDistance = autoDrive.autoRoute.mineralMoveDistance[mineralPosIndex] * -1.0f;
        autoDrive.driveState = Drive.START;
        return WAIT_FOR_KNOCKOUT_MOVE_BACK;
      }
    },
    WAIT_FOR_KNOCKOUT_MOVE_BACK{
      public AutoState update(MyAutoDrive autoDrive, Telemetry telemetry) {
        if (autoDrive.driveState != Drive.IDLE){
          autoDrive.driveState = autoDrive.driveState.update(autoDrive);
          return WAIT_FOR_KNOCKOUT_MOVE_BACK;
        } else {
          return IDLE;
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
        return TURN;
      }
    },
    TURN {
      public Rotate update(MyAutoDrive op) {
        op.gyro.update(op.telemetry);
        op.robotPos.angle = op.gyro.getHeading();
        op.angleDelta = Gyro.convertAngle180(op.robotPos.angle - op.targetAngle);
        if (Math.abs(op.angleDelta) < 1.0f) {
          op.rotPower = 0.0;
          op.mecanum.stop();
          return IDLE;
        } else {
          op.rotPower = Math.signum(op.angleDelta) * Wheels.AUTO_ROT_POW;
          //if (Math.abs(op.angleDelta) < 15.0f) {
          //  op.rotPower = ((Wheels.AUTO_ROT_POW * 0.5)* (Math.abs(op.angleDelta) / 15.0f)) + (Wheels.AUTO_ROT_POW * 0.5);
          //}
          op.mecanum.drive(0.0, 0.0f, op.rotPower, op.telemetry);
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
          op.drivePower = Wheels.AUTO_DR_POW * Math.signum(op.driveDistance);
          op.mecanum.drive(op.drivePower, 0.0, 0.0, op.telemetry);
          return MOVE;
        }
      }
    },
    MOVE {
      public Drive update(MyAutoDrive op) {
        // watch encoder count, with in range, stop wheels
        if (op.mecanum.isTargetDriveDone()) {
          op.drivePower = 0.0;
          op.mecanum.stop();
          return IDLE;
        } else {
          float distRemain = op.mecanum.getRemainingDistance();
          if (distRemain < 6.0f) {
            op.drivePower = Wheels.AUTO_DR_POW * Math.signum(op.driveDistance);
            op.drivePower = ((op.drivePower * 0.5) * (distRemain / 6.0f)) + (op.drivePower * 0.5);
            op.mecanum.drive(op.drivePower, 0.0, 0.0, op.telemetry);
          }
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
        return ORIENT_TO_TARGET;
      }
    },
    ORIENT_TO_TARGET {
      public GoToTargetPos update(MyAutoDrive op) {
        if (op.rotState != Rotate.IDLE) {
          op.rotState = op.rotState.update(op);
          return ORIENT_TO_TARGET;
        } else {
          op.driveState = Drive.START;
          return MOVE_TO_TARGET;
        }
      }
    },
    MOVE_TO_TARGET {
      public GoToTargetPos update(MyAutoDrive op) {
        if (op.driveState != Drive.IDLE){
          op.driveState = op.driveState.update(op);
          return  MOVE_TO_TARGET;
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
    gyro.updateHeadingOffset(autoRoute.landingPos.angle);
    robotPos.setPos(autoRoute.landingPos);
    autoState = AutoState.PREP_TFO_SCAN;
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

    telemetry.addData("AutoState", "auto %s tgt %s drive %s rot %s vuMark %s GoldPos %s",
      autoState, gotoTargetState, driveState, rotState, nav.getVisibleTarget(), goldMineralPos);
    telemetry.addData("AutoDrive", "POS target %s cur %s target angle %.1f dist %.1f",
      targetPos, robotPos, targetAngle, driveDistance);
    telemetry.addData("AutoDriveDebug", "drPow %.1f rotPow %.1f deltaAngle %.1f",
      drivePower, rotPower, angleDelta);
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