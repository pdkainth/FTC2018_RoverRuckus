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
          return IDLE;
        }
      }
    };

    public abstract AutoState update(MyAutoDrive autoDrive, Telemetry telemetry);
  } // end of AutoState enum

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
    autoState = AutoState.START;
    telemetry.addData("AutoStatus", "Started A %s %s states auto %s",
        allianceColor, allianceSide, autoState);
 }

  public void loop() {
    autoState = autoState.update(this, telemetry);

    telemetry.addData("AutoStatus", "runtime: %s A %s %s states auto %s",
      runtime.toString(), allianceColor, allianceSide, autoState);

    intake.update(telemetry);
    minDel.update(telemetry);
    lift.update(telemetry);
  }

  public void stop() {
    mecanum.stop();
    intake.stop();
    minDel.stop();
    lift.stop();
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