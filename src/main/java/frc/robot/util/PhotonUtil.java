package frc.robot.util;

import java.time.Duration;

public class PhotonUtil {
    /**
     * Restarts PhotonVision.
     */
    public static void restartPhotonVisionService() {
        ProcessBuilder builder = new ProcessBuilder(
            "ssh",
            "photon@10.43.22.11",
            "sudo systemctl restart photonvision.service"
        );
        try {
            builder.start().waitFor(Duration.ofSeconds(5));
        } catch (Exception e) {
            System.err.println("Error: " + e.toString());
        }
    } 
}
