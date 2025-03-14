package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/* RELEVANT LINKS
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
 * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
 */

public class Odometry {
  // // Locations for the swerve drive modules relative to the robot center.
  // private Translation2d m_frontLeftLocation = new Translation2d(-11.5, 10.5);
  // private Translation2d m_frontRightLocation = new Translation2d(11.5, 10.5);
  // private Translation2d m_backLeftLocation = new Translation2d(-11.5, -10.5);
  // private Translation2d m_backRightLocation = new Translation2d(11.5, -10.5);

  // // Creating kinematics object using the module locations
  // private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  //   m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  // );

  // // private SwerveDrivePoseEstimator m_odometry; // comes from subsystem -> can't tell it what we want, it estimates where the Robot is

  // SwerveDriveOdometry m_odometry;

  // public Odometry( SwerveModulePosition m_frontLeftModule, SwerveModulePosition m_frontRightModule, SwerveModulePosition m_backLeftModule, SwerveModulePosition m_backRightModule) {
  //   // Creating my odometry object from the kinematics object and the initial wheel positions.
  //   // Here, our starting pose is 5 meters along the long end of the field and in the
  //   // center of the field along the short end, facing the opposing alliance wall.
  //   m_odometry = new SwerveDriveOdometry(
  //     m_kinematics, m_gyro.getRotation2d(),
  //     new SwerveModulePosition[] {
  //       m_frontLeftModule.getPosition(),
  //       m_frontRightModule.getPosition(),
  //       m_backLeftModule.getPosition(),
  //       m_backRightModule.getPosition()
  //     }, new Pose2d(5.0, 13.5, new Rotation2d()));
  // }

  public Odometry(){}
    
}
