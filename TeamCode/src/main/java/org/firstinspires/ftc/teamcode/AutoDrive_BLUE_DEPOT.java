package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Drive : BLUE DEPOT")
public class AutoDrive_BLUE_DEPOT extends MyAutoDrive {

  @Override
  public void init() {
    setAlliance(AllianceColor.BLUE, AllianceSide.DEPOT);
    super.init();
  }

  public void start(){
    super.start();

  }

}
