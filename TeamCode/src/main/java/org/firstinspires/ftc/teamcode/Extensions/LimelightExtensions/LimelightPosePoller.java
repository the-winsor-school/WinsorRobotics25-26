package org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Background poller that fetches Limelight fiducials on a fixed interval.
 */
public class LimelightPosePoller implements AutoCloseable {
    private final OpMode opMode;
    private final LimelightResultsClient client;
    private final long pollIntervalMs;
    private final AtomicReference<List<AprilTagPoseData>> latest;
    private final AtomicReference<IOException> lastError;

    private volatile boolean running;
    private Thread worker;

    public LimelightPosePoller(OpMode opMode, LimelightResultsClient client, long pollIntervalMs) {
        this.opMode = opMode;
        this.client = client;
        this.pollIntervalMs = pollIntervalMs;
        this.latest = new AtomicReference<>(Collections.emptyList());
        this.lastError = new AtomicReference<>(null);
    }

    /**
     * Starts the background polling thread if not already running.
     */
    public synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        worker = new Thread(this::runLoop, "LimelightPosePoller");
        worker.setDaemon(true);
        worker.start();
    }

    /**
     * Stops the background polling thread.
     */
    public synchronized void stop() {
        running = false;
        if (worker != null) {
            worker.interrupt();
        }
    }

    @Override
    public void close() {
        stop();
    }

    /**
     * Returns the most recently fetched fiducials (thread-safe snapshot).
     */
    public List<AprilTagPoseData> getLatest() {
        return latest.get();
    }

    /**
     * Returns the first fiducial with the given tag ID from the latest snapshot.
     *
     * @param id AprilTag ID to search for.
     * @return the matching tag, or null if not present.
     */
    public AprilTagPoseData tryGetTagId(int id) {
        List<AprilTagPoseData> snapshot = latest.get();
        for (AprilTagPoseData tag : snapshot) {
            if (tag.fID == id) {
                return tag;
            }
        }
        return null;
    }

    /**
     * Returns the most recent IOException encountered during polling, or null.
     */
    public IOException getLastError() {
        return lastError.get();
    }

    private void runLoop() {
        while (running && opMode.opModeIsActive()) {
            long loopStart = System.currentTimeMillis();
            try {
                latest.set(client.getFiducials());
                lastError.set(null);
            } catch (IOException e) {
                lastError.set(e);
            }
            long elapsed = System.currentTimeMillis() - loopStart;
            long sleepMs = pollIntervalMs - elapsed;
            if (sleepMs > 0) {
                try {
                    Thread.sleep(sleepMs);
                } catch (InterruptedException ignored) {
                    // allow thread to exit on stop()
                }
            }
        }
    }
}
