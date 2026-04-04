// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PIDsAndThings;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.DrivetrainPIDConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.commands.DrivetrainPIDController;

public class PIDToPositionSpline extends Command {
    private final SwerveDrivetrain drive;
    private final DrivetrainPIDController controller;
    private Pose2d setpoint;  
    private ArrayList<Pose2d> waypoints; 
    private int waypointIndex = 0;
    private static final double tolerance = 0.2;  
    private Pose2d currentTarget;
    private double distance = 0.0;
    private double lookAhead = 0.5;

    public PIDToPositionSpline(SwerveDrivetrain drive,  ArrayList<Pose2d> waypoints) {
        this(drive, waypoints, DrivetrainConstants.autoPidToPositionConstants);
    }

    public PIDToPositionSpline(SwerveDrivetrain drive, ArrayList<Pose2d> waypoints, DrivetrainPIDConstants constants) {
        this.drive = drive;
        this.controller = new DrivetrainPIDController(constants);
        this.waypoints = waypoints;

        addRequirements(drive);
    }

    public Pose2d backAway(Pose2d pose, double distance) {
        var angle = pose.getRotation();

        Translation2d vec = new Translation2d(Units.inchesToMeters(distance), 0.0);
        vec = vec.rotateBy(angle);

        return new Pose2d(pose.getX() + vec.getX(), pose.getY() + vec.getY(), pose.getRotation());
    }

    public double getDistance(Pose2d a, Pose2d b) {
        return Math.hypot((a.getX()-b.getX()), (a.getY()-b.getY()));
    }

    @Override
    public void initialize() {

        if (waypoints == null || waypoints.isEmpty()){
            setpoint = drive.getPose();
            return;
        }
        waypointIndex = 0;
        setpoint = waypoints.get(0);
        controller.reset(drive.getPose(), ChassisSpeeds.fromFieldRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRawRotation2d()));
       
    }

    @Override
    public void execute() {
       
        if (waypoints == null || waypoints.isEmpty()){
            drive.stop();
            return;
        }

            currentTarget = waypoints.get(waypointIndex);
            distance = getDistance(drive.getPose(), currentTarget);

            if (distance < tolerance){
                waypointIndex++;
                currentTarget = waypoints.get(waypointIndex);
             
            }

        ChassisSpeeds speeds = controller.calculate(drive.getPose(), currentTarget);

        drive.fieldOrientedDrive(speeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
         if (waypoints == null || waypoints.isEmpty()) return true;
            Pose2d finalTarget = waypoints.get(waypoints.size() - 1);
            return getDistance(drive.getPose(), finalTarget) < tolerance;
        }
}