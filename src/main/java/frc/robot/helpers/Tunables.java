package frc.robot.helpers;

import frc.robot.helpers.Tuner.TunableDouble;
import frc.robot.helpers.Tuner.TunableInteger;

public final class Tunables {
  // public static final TunableDouble kP = new TunableDouble("kP", 0.04);
  // public static final TunableDouble kF = new TunableDouble("kF", 0.05);
  // public static final TunableDouble kI = new TunableDouble("kI", 0.000001);
  // public static final TunableDouble kD = new TunableDouble("kD", 0);

  public static final TunableDouble kP = new TunableDouble("kP", 0.02);
  public static final TunableDouble kF = new TunableDouble("kF", 0.0);
  public static final TunableDouble kI = new TunableDouble("kI", 0.0);
  public static final TunableDouble kD = new TunableDouble("kD", 0.0);
  public static final TunableInteger fastAndFaster = new TunableInteger("Speed", 30000);
  public static final TunableInteger monsterInject = new TunableInteger("Acceleration", 25000);
  public static final TunableInteger kSmoothing = new TunableInteger("kSmoothing", 1);
  public static final TunableDouble kScoreMidTop = new TunableDouble("kScoreMidTop", -1);
  public static final TunableDouble kScoreMidBottom = new TunableDouble("kScoreMidBottom", -1);
  public static final TunableDouble kScoreGroundTop = new TunableDouble("kScoreGroundTop", -0.25);
  public static final TunableDouble kScoreGroundBottom = new TunableDouble("kScoreGroundBottom", 0.1);
  public static final TunableDouble kYeetPartyTop = new TunableDouble("kYeetPartyTop", -0.75);
  public static final TunableDouble kYeetPartyBottom = new TunableDouble("kYeetPartyBottom", -1);
  public static final TunableInteger kPartyCounts = new TunableInteger("kPartyCounts", 8750);
  public static final TunableInteger kLowCounts = new TunableInteger("kLowCounts", 11975);
  public static final TunableInteger kMidCounts = new TunableInteger("kMidCounts", 8000);
  public static final TunableInteger kSubstationCounts = new TunableInteger("kSubstationCounts", 10000);

}
