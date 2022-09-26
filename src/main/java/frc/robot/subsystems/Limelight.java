// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  double x;
  double y;
  double a;
  double v;

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    v = tv.getDouble(0.0);
  }

  public double getX() {
    x = tx.getDouble(0.0);
    return x;
  }

  public double getY() {
    y = ty.getDouble(0.0);
    return y;
  }

  public double getV() {
    v = tv.getDouble(0.0);
    return v;
  }
}
