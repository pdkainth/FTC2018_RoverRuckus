package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing TFO teleOP")

public class Test_TFO_TeleOp extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private MineralDetect mineralDetect = new MineralDetect();

  @Override
  public void init(){
    mineralDetect.init(hardwareMap);
  }

  @Override
  public void init_loop(){}

  @Override
  public void start(){
    runtime.reset();
    mineralDetect.start(telemetry);
  }

  @Override
  public void loop(){

    telemetry.addData("Status", "Initialized");
    mineralDetect.detect(telemetry);
  }

  @Override
  public void stop(){
    mineralDetect.stop();
  }
}
