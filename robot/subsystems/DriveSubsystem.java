package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeftModule=new SwerveModule(
        DriveConstants.FLDrivePort, 
        DriveConstants.FLTurnPort, 
        DriveConstants.FLAbsEncoderPort, 
        DriveConstants.FLAbsEncoderOffset, 
        DriveConstants.FLDriveReversed, 
        DriveConstants.FLTurnReversed, 
        DriveConstants.FLAbsEncoderReversed);

    private final SwerveModule frontRightModule=new SwerveModule(
        DriveConstants.FRDrivePort, 
        DriveConstants.FRTurnPort, 
        DriveConstants.FRAbsEncoderPort, 
        DriveConstants.FRAbsEncoderOffset, 
        DriveConstants.FRDriveReversed, 
        DriveConstants.FRTurnReversed, 
        DriveConstants.FRAbsEncoderReversed);

    private final SwerveModule backLeftModule=new SwerveModule(
        DriveConstants.BLDrivePort, 
        DriveConstants.BLTurnPort, 
        DriveConstants.BLAbsEncoderPort, 
        DriveConstants.BLAbsEncoderOffset, 
        DriveConstants.BLDriveReversed, 
        DriveConstants.BLTurnReversed, 
        DriveConstants.BLAbsEncoderReversed);

    private final SwerveModule backRightModule=new SwerveModule(
        DriveConstants.BRDrivePort, 
        DriveConstants.BRTurnPort, 
        DriveConstants.BRAbsEncoderPort, 
        DriveConstants.BRAbsEncoderOffset, 
        DriveConstants.BRDriveReversed, 
        DriveConstants.BRTurnReversed, 
        DriveConstants.BRAbsEncoderReversed);

    private AHRS gyro=new AHRS(SPI.Port.kMXP);

    //I know 0 AHRS Navx code so I coppied this word for word bar for bar from YT
    //this gives the gyroscope some time to recalibrate when it boots up before resetting
    public DriveSubsystem(){
        new Thread(() -> {
            try{
                Thread.sleep(1000);
            } catch(Exception e) {
                }
        }).start();
    }

    //the only one I understand. Sets the gyro heading to 0 (essentially makes the current direction forward to the robot)
    public void zeroHeading(){
        gyro.reset();
    }

    //gets current angle clamped between +-180 degrees
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //converts current angle to a Rotation2d object
    public Rotation2d getRotation2d(){
        Rotation2d temp=new Rotation2d();
        return temp.fromDegrees(getHeading());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void stopModules(){
        frontLeftModule.STOPRIGHTNOWRAAAAAH();
        frontRightModule.STOPRIGHTNOWRAAAAAH();
        backLeftModule.STOPRIGHTNOWRAAAAAH();
        backRightModule.STOPRIGHTNOWRAAAAAH();
    }

    //takes an array of swerve module states and uses it to set the states of all 4 modules
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        backLeftModule.setDesiredState(desiredStates[2]);
        backRightModule.setDesiredState(desiredStates[3]);
    }
}
