package org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Simple HTTP client for Limelight's /results endpoint.
 * <p>
 * Usage:
 * <pre>
 * LimelightResultsClient client = new LimelightResultsClient();
 * List&lt;AprilTagPoseData&gt; tags = client.getFiducials();
 * </pre>
 * Call from a background thread to avoid NetworkOnMainThreadException.
 */
public class LimelightResultsClient {
    public static final String DEFAULT_RESULTS_URL = "http://172.28.0.1:5807/results";

    private final String resultsUrl;
    private int connectTimeoutMs = 250;
    private int readTimeoutMs = 500;

    public LimelightResultsClient() {
        this(DEFAULT_RESULTS_URL);
    }

    public LimelightResultsClient(String resultsUrl) {
        this.resultsUrl = resultsUrl;
    }

    public void setConnectTimeoutMs(int connectTimeoutMs) {
        this.connectTimeoutMs = connectTimeoutMs;
    }

    public void setReadTimeoutMs(int readTimeoutMs) {
        this.readTimeoutMs = readTimeoutMs;
    }

    /**
     * Fetches /results and returns the parsed Fiducial list.
     */
    public List<AprilTagPoseData> getFiducials() throws IOException {
        String json = fetchResultsJson();
        return parseFiducials(json);
    }

    /**
     * Parses a raw /results JSON string and extracts the Fiducial list.
     */
    public List<AprilTagPoseData> parseFiducials(String resultsJson) {
        if (resultsJson == null || resultsJson.isEmpty()) {
            return Collections.emptyList();
        }
        JSONObject root = new JSONObject(resultsJson);
        JSONArray fiducials = root.optJSONArray("Fiducial");
        if (fiducials == null || fiducials.length() == 0) {
            return Collections.emptyList();
        }
        List<AprilTagPoseData> result = new ArrayList<>(fiducials.length());
        for (int i = 0; i < fiducials.length(); i++) {
            JSONObject item = fiducials.optJSONObject(i);
            if (item != null) {
                result.add(AprilTagPoseData.fromJson(item));
            }
        }
        return result;
    }

    private String fetchResultsJson() throws IOException {
        HttpURLConnection connection = null;
        try {
            URL url = new URL(resultsUrl);
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(connectTimeoutMs);
            connection.setReadTimeout(readTimeoutMs);
            connection.setUseCaches(false);
            connection.setDoInput(true);

            try (BufferedReader reader = new BufferedReader(
                    new InputStreamReader(connection.getInputStream()))) {
                StringBuilder builder = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    builder.append(line);
                }
                return builder.toString();
            }
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }
}
