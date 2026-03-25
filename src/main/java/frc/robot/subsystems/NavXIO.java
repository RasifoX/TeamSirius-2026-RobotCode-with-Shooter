package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * NAVX GYRO uygulaması
 *
 * NavX MXP (RoboRIO MXP port) üzerinden bağlanır.
 * Studica 2026 kütüphanesini kullanır.
 *
 * Kullanmak için Constants.java → DriveConstants.kUseNavX = true yap.
 * vendordeps klasörüne Studica.json eklenmiş olmalı (zaten mevcut görünüyor).
 */
public class NavXIO implements GyroIO {

  private final AHRS m_navX;

  public NavXIO() {
    // MXP portu: RoboRIO'nun genişleme portuna takılı NavX için
    m_navX = new AHRS(NavXComType.kMXP_SPI);

    // NavX kalibre edilene kadar kısa bekle (max 2 saniye)
    long start = System.currentTimeMillis();
    while (m_navX.isCalibrating() && (System.currentTimeMillis() - start) < 2000) {
      try { Thread.sleep(10); } catch (InterruptedException ignored) {}
    }

    m_navX.reset();
  }

  @Override
  public Rotation2d getRotation2d() {
    // NavX CW pozitif döner, WPILib CCW pozitif ister → negatif
    return Rotation2d.fromDegrees(-m_navX.getYaw());
  }

  @Override
  public void reset() {
    m_navX.reset();
  }

  @Override
  public boolean isConnected() {
    return m_navX.isConnected();
  }
}