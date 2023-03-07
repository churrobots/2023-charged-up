package frc.robot.helpers;

import frc.robot.helpers.Tuner.TunableBoolean;
import frc.robot.helpers.Tuner.TunableDouble;
import frc.robot.helpers.Tuner.TunableInteger;

public final class Tunables {
  public static final TunableDouble kP = new TunableDouble("kP", 0.03);
  public static final TunableDouble kF = new TunableDouble("kF", 0.03);
  public static final TunableDouble kI = new TunableDouble("kI", 0);
  public static final TunableDouble kD = new TunableDouble("kD", 0);
  public static final TunableInteger kachow = new TunableInteger("Speed", 18000);
  public static final TunableInteger kaching = new TunableInteger("Acceleration", 8000);

}
