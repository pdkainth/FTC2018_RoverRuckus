package org.firstinspires.ftc.teamcode;

public class AutoRoute {

  public static final int NUM_MINERAL_POS = 3;
  public static final int NUM_DEPOT_ROUTE_POS = 3;


  Position landingPos = new Position();
  Position[] mineralPos = new Position[NUM_MINERAL_POS];
  float[] mineralMoveDistance = new float[NUM_MINERAL_POS];
  Position[] depotRoutePos = new Position[NUM_DEPOT_ROUTE_POS];
  Position craterPos = new Position();

  float vuforiaAngle;

  public AutoRoute()
  {
    for (int index = 0; index < NUM_MINERAL_POS; index++){
      mineralPos[index] = new Position();
    }
    for (int index = 0; index < NUM_DEPOT_ROUTE_POS; index++){
      depotRoutePos[index] = new Position();
    }

  }
}

