package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Drive : RED DEPOT")
public class AutoDrive_RED_DEPOT extends MyAutoDrive {

  @Override
  public void init() {
    setAlliance(AllianceColor.RED, AllianceSide.DEPOT);
    super.init();


  }
}
