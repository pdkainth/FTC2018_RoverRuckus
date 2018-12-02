package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.MineralDeliver;
import org.firstinspires.ftc.teamcode.MineralDetect;
import org.firstinspires.ftc.teamcode.Wheels;
import org.firstinspires.ftc.teamcode.Lift;

@TeleOp(name="Testing teleOP")

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
