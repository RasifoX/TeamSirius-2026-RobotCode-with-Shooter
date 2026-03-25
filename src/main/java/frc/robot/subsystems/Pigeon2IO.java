package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * PIGEON 2.0 GYRO uygulaması
 *
 * Phoenix 6 API kullanır.
 * CCW pozitif yaw verir — WPILib standardıyla uyumludur.
 */
public class Pigeon2IO implements GyroIO {

  private final Pigeon2 m_pigeon;

  public Pigeon2IO(int canId) {
    m_pigeon = new Pigeon2(canId);
    m_pigeon.reset();
  }

  @Override
  public Rotation2d getRotation2d() {
    // Phoenix 6: getYaw() → derece cinsinden döner.
    // Rotation2d.fromDegrees() otomatik normalize eder.
    return m_pigeon.getRotation2d();
  }

  @Override
  public void reset() {
    m_pigeon.reset();
  }

  @Override
  public boolean isConnected() {
    // Phoenix 6: getFaultField() veya version sinyali üzerinden bağlantı kontrolü
    return m_pigeon.getVersion().getValueAsDouble() > 0;
  }
}