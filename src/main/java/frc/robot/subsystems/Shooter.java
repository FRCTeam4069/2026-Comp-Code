package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    SparkFlex shooterOneLeft, shooterOneRight, shooterTwoLeft, shooterTwoRight;

    public Shooter(){

        shooterOneLeft = new SparkFlex(DeviceIDs.SHOOTER_ONE_LEFT, MotorType.kBrushless);
        shooterOneRight = new SparkFlex(DeviceIDs.SHOOTER_ONE_RIGHT, MotorType.kBrushless);
        shooterTwoLeft = new SparkFlex(DeviceIDs.SHOOTER_TWO_LEFT, MotorType.kBrushless);
        shooterTwoRight = new SparkFlex(DeviceIDs.SHOOTER_TWO_RIGHT, MotorType.kBrushless);

        shooterOneLeft.configure(ShooterConstants.shooterOneLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterOneRight.configure(ShooterConstants.shooterOneRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoLeft.configure(ShooterConstants.shooterTwoLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterTwoRight.configure(ShooterConstants.shooterTwoRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        






    }

    
}
