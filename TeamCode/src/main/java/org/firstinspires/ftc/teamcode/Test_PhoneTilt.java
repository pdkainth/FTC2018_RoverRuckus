package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing Phone Tilt")

public class Test_PhoneTilt extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private PhoneTilt phoneTilt = new PhoneTilt();

  @Override
  public void init(){
    phoneTilt.init(hardwareMap);
  }

  @Override
  public void init_loop(){}

  @Override
  public void start(){
    runtime.reset();
  }

  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    double phoneTilePos = (-gamepad1.left_stick_y + 1.0) / 2.0;
    phoneTilt.setPhoneTiltPos(phoneTilePos);
    phoneTilt.update(telemetry);
  }

  @Override
  public void stop(){
    phoneTilt.stop();
  }
}
