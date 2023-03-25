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

  public static final TunableInteger kArmSpeed = new TunableInteger("kArmSpeed", 30000);
  public static final TunableInteger kArmAcceleration = new TunableInteger("kArmAcceleration", 25000);
  public static final TunableInteger kArmSmoothing = new TunableInteger("kArmSmoothing", 1);

  public static final TunableDouble kScoreMidTopRollerSpeed = new TunableDouble("kScoreMidTopRollerSpeed", -1);
  public static final TunableDouble kScoreMidBottomRollerSpeed = new TunableDouble("kScoreMidBottomRollerSpeed", -1);

  public static final TunableDouble kScoreGroundTopRollerSpeed = new TunableDouble("kScoreGroundTopRollerSpeed", -0.4);
  public static final TunableDouble kScoreGroundBottomRollerSpeed = new TunableDouble("kScoreGroundBottomRollerSpeed",
      -0.4);
  public static final TunableDouble kYeetPartyTopRollerSpeed = new TunableDouble("kYeetPartyTopRollerSpeed", -1);
  public static final TunableDouble kYeetPartyBottomRollerSpeed = new TunableDouble("kYeetPartyBottomRollerSpeed", -1);

  public static final TunableInteger kPartyCounts = new TunableInteger("kPartyCounts", 8750);
  public static final TunableInteger kLowCounts = new TunableInteger("kLowCounts", 11975);
  public static final TunableInteger kMidCounts = new TunableInteger("kMidCounts", 8000);
  public static final TunableInteger kSubstationCounts = new TunableInteger("kSubstationCounts", 10000);

}
