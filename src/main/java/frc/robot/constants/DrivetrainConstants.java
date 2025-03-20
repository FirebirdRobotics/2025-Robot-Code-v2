// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class DrivetrainConstants {
            public static final LinearVelocity kMaxSpeed = FeetPerSecond.of(15);
        public static final AngularVelocity kMaxRotationRate = RotationsPerSecond.of(1);

        public static final ProfiledPIDController kXController = new ProfiledPIDController(10, 0, 0.3, new Constraints(Units.feetToMeters(10), 2)); // Forward/back
        public static final ProfiledPIDController kYController = new ProfiledPIDController(10, 0, 0.3, new Constraints(Units.feetToMeters(10), 2)); // Left/right
        public static final ProfiledPIDController kHeadingController = new ProfiledPIDController(7, 0, 0, new Constraints(Units.rotationsToRadians(3), 6)); // rotation

        public static final PIDConstants kTranslationConstants = new PIDConstants(8, 0.3);
        public static final PIDConstants kHeadingConstants = new PIDConstants(7, 0);

        public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeed, MetersPerSecondPerSecond.of(3), kMaxRotationRate, RotationsPerSecondPerSecond.of(3));
}
