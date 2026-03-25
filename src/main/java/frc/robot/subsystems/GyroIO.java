package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * GYRO ARAYÜZÜ (Strateji Deseni)
 *
 * DriveSubsystem gyronun fiziksel tipini bilmez.
 * Sadece bu üç metodu çağırır.
 * Pigeon ↔ NavX geçişi için sadece DriveConstants.kUseNavX değiştirilir.
 */
public interface GyroIO {

  /** Robotun sahaya göre açısını Rotation2d olarak döndürür. CCW pozitif. */
  Rotation2d getRotation2d();

  /** Gyro'yu sıfırlar (o an baktığı yön = 0°). */
  void reset();

  /** Gyro'nun bağlı ve veri üretiyor olduğunu kontrol eder. */
  boolean isConnected();
}