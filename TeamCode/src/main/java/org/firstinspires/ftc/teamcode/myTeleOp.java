package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MineralDeliver;
import org.firstinspires.ftc.teamcode.Wheels;
import org.firstinspires.ftc.teamcode.Lift;

@TeleOp(name="Main teleOP")

public class myTeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private Wheels mecanum = new Wheels();
  private Intake intake = new Intake();
  private MineralDeliver minDel = new MineralDeliver();
  private Lift lift = new Lift();
  private Gyro gyro = new Gyro();

  @Override
  public void init(){
    mecanum.init(hardwareMap);
    intake.init(hardwareMap);
    minDel.init(hardwareMap);
    lift.init(hardwareMap);
    gyro.init(hardwareMap);
  }

  @Override
  public void init_loop() {
    intake.update(telemetry);
  }

  @Override
  public void start(){
    runtime.reset();
  }

  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    updateWheels();
    updateIntake();
    updateMineral();
    updateLift();
    gyro.update(telemetry);
  }

  @Override
  public void stop(){
    mecanum.stop();
    intake.stop();
    minDel.stop();
    lift.stop();
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

  public void updateIntake(){

    boolean on = gamepad1.right_bumper;
    boolean off = gamepad1.left_bumper;

    if(on){
      intake.turnOn();
    } else if(off){
      intake.turnOff();
    }

    if (intake.isOff()) {
      if (gamepad1.left_trigger > 0.3) {
        intake.pulse(-gamepad1.left_trigger);
      } else if (gamepad1.right_trigger > 0.3) {
        intake.pulse(gamepad1.right_trigger );
      }
    }

    boolean up = gamepad1.y;
    boolean down = gamepad1.a;

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

    if (gamepad2.b == true) {
      lift.liftToPosition(Lift.LIFT_ENC_POS_HANG);
    } else if (gamepad2.a == true) {
      lift.liftToPosition(Lift.LIFT_ENC_POS_MIN);
    }

    lift.update(telemetry);
  }
}
