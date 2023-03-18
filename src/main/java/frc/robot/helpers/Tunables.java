package frc.robot.helpers;

import frc.robot.helpers.Tuner.TunableDouble;
import frc.robot.helpers.Tuner.TunableInteger;

public final class Tunables {
  // public static final TunableDouble kP = new TunableDouble("kP", 0.04);
  // public static final TunableDouble kF = new TunableDouble("kF", 0.05);
  // public static final TunableDouble kI = new TunableDouble("kI", 0.000001);
  // public static final TunableDouble kD = new TunableDouble("kD", 0);
  public static final TunableDouble kP = new TunableDouble("kP", 0.04);
  public static final TunableDouble kF = new TunableDouble("kF", 0.0);
  public static final TunableDouble kI = new TunableDouble("kI", 0.0);
  public static final TunableDouble kD = new TunableDouble("kD", 0.0);
  public static final TunableInteger fastAndFaster = new TunableInteger("Speed", 30000);
  public static final TunableInteger monsterInject = new TunableInteger("Acceleration", 25000);
  public static final TunableInteger kSmoothing = new TunableInteger("kSmoothing", 1);
}
