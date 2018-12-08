package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyAutoDrive {

  public enum AllianceColor {UNKNOWN, RED, BLUE}

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

  private AllianceColor allianceColor;

  private AutoState autoState;

  public void init(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor allianceColor) {
    telemetry.addData("AutoStatus", "Initializing");

    this.allianceColor = allianceColor;

    mecanum.init(hardwareMap);
    intake.init(hardwareMap);
    minDel.init(hardwareMap);
    lift.init(hardwareMap);

    autoState = AutoState.IDLE;

    telemetry.addData("AutoStatus", "Initialized A %s", this.allianceColor);
  }

  public void init_loop(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
    updateWheels(telemetry, gamepad1);
    updateIntake(telemetry, gamepad1);
    updateMineral(telemetry, gamepad2);
    updateLift(telemetry, gamepad2);
  }

  public void start(Telemetry telemetry) {
    runtime.reset();
    autoState = AutoState.START;
    telemetry.addData("AutoStatus", "Started F %s A %s", allianceColor);
 }

  public void loop(Telemetry telemetry) {
    autoState = autoState.update(this, telemetry);

    telemetry.addData("AutoStatus", "runtime: %s A %sstates auto %s",
      runtime.toString(), allianceColor, autoState);

    intake.update(telemetry);
    minDel.update(telemetry);
    lift.update(telemetry);
  }

  public void stop() {
    mecanum.stop();
    intake.stop();
    minDel.stop();
  }

  public void updateWheels(Telemetry telemetry, Gamepad gamepad1){

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

  public void updateIntake(Telemetry telemetry, Gamepad gamepad1){

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

  public void updateMineral(Telemetry telemetry, Gamepad gamepad2){
    double mineralMotorPower = -gamepad2.left_stick_y;
    minDel.upDn(mineralMotorPower);
    if(gamepad2.left_bumper){
      minDel.collect();
    } else if(gamepad2.right_bumper){
      minDel.dump();
    }
    minDel.update(telemetry);
  }

  public void updateLift(Telemetry telemetry, Gamepad gamepad2){

    double liftPower = -gamepad2.right_stick_y;
    lift.lift(liftPower);
    lift.update(telemetry);
  }
}