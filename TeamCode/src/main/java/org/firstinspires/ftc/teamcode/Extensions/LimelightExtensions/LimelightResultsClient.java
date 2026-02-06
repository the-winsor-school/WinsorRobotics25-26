package org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONException;

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
    private final String baseUrl;
    private int connectTimeoutMs = 1000;
    private int readTimeoutMs = 1000;

    /**
     * Builds a client using the given Limelight IP (or hostname) and the default port 5807.
     *
     * @param ipAddress e.g. "172.28.0.1" or "limelight.local"
     */
    public LimelightResultsClient(String ipAddress) {
        this(ipAddress, 5807);
    }

    /**
     * Builds a client using the given IP/host and port, targeting /results.
     *
     * @param ipAddress e.g. "172.28.0.1" or "limelight.local"
     * @param port      HTTP port configured on the Limelight
     */
    public LimelightResultsClient(String ipAddress, int port) {
        baseUrl = "http://" + ipAddress + ":" + port;
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
        JSONObject root;
        try {
            root = new JSONObject(resultsJson);
        } catch (JSONException e) {
            // Malformed JSON; treat as no detections.
            return Collections.emptyList();
        }
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

    /**
     * Switches to a different pipeline by index.
     */
    public String switchPipeline(int index) throws IOException {
        return postText("/pipeline-switch?index=" + index, null, "text/plain");
    }

    /**
     * Forces the camera to reload the current pipeline and all pipeline resources.
     */
    public String reloadPipeline() throws IOException {
        return postText("/reload-pipeline", null, "text/plain");
    }

    /**
     * Returns JSON of the default calibration result.
     */
    public String getCalibrationDefault() throws IOException {
        return getText("/cal-default");
    }

    /**
     * Returns JSON of the custom calibration result (file system).
     */
    public String getCalibrationFile() throws IOException {
        return getText("/cal-file");
    }

    /**
     * Returns JSON of the custom calibration result (eeprom).
     */
    public String getCalibrationEeprom() throws IOException {
        return getText("/cal-eeprom");
    }

    /**
     * Returns JSON of the latest calibration result.
     */
    public String getCalibrationLatest() throws IOException {
        return getText("/cal-latest");
    }

    /**
     * Returns JSON calibration stats (reprojection errors, point counts, etc.).
     */
    public String getCalibrationStats() throws IOException {
        return getText("/cal-stats");
    }

    /**
     * Downloads the calibration point cloud as a PLY file (binary).
     */
    public byte[] getCalibrationPointCloud() throws IOException {
        return getBytes("/cal-pointcloud");
    }

    /**
     * Updates the calibration result stored in EEPROM.
     * Provide the JSON calibration data in the request body.
     */
    public String postCalibrationEeprom(String calibrationJson) throws IOException {
        return postText("/cal-eeprom", calibrationJson, "application/json");
    }

    /**
     * Updates the calibration result stored in the file system.
     * Provide the JSON calibration data in the request body.
     */
    public String postCalibrationFile(String calibrationJson) throws IOException {
        return postText("/cal-file", calibrationJson, "application/json");
    }

    private String fetchResultsJson() throws IOException {
        return getText("/results");
    }

    private String getText(String path) throws IOException {
        HttpURLConnection connection = null;
        try {
            URL url = new URL(buildUrl(path));
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

    private byte[] getBytes(String path) throws IOException {
        HttpURLConnection connection = null;
        try {
            URL url = new URL(buildUrl(path));
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            connection.setConnectTimeout(connectTimeoutMs);
            connection.setReadTimeout(readTimeoutMs);
            connection.setUseCaches(false);
            connection.setDoInput(true);

            int estimated = connection.getContentLength();
            if (estimated < 0) {
                estimated = 4096;
            }
            byte[] buffer = new byte[estimated];
            int total = 0;
            try (java.io.InputStream in = connection.getInputStream()) {
                int read;
                while ((read = in.read(buffer, total, buffer.length - total)) != -1) {
                    total += read;
                    if (total == buffer.length) {
                        byte[] grown = new byte[buffer.length * 2];
                        System.arraycopy(buffer, 0, grown, 0, buffer.length);
                        buffer = grown;
                    }
                }
            }
            byte[] result = new byte[total];
            System.arraycopy(buffer, 0, result, 0, total);
            return result;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    private String postText(String path, String body, String contentType) throws IOException {
        HttpURLConnection connection = null;
        try {
            URL url = new URL(buildUrl(path));
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("POST");
            connection.setConnectTimeout(connectTimeoutMs);
            connection.setReadTimeout(readTimeoutMs);
            connection.setUseCaches(false);
            connection.setDoInput(true);
            if (body != null) {
                connection.setDoOutput(true);
                if (contentType != null && !contentType.isEmpty()) {
                    connection.setRequestProperty("Content-Type", contentType);
                }
                byte[] bytes = body.getBytes(java.nio.charset.StandardCharsets.UTF_8);
                connection.setFixedLengthStreamingMode(bytes.length);
                try (java.io.OutputStream out = connection.getOutputStream()) {
                    out.write(bytes);
                }
            }

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

    private String buildUrl(String path) {
        if (path == null || path.isEmpty()) {
            return baseUrl;
        }
        if (path.startsWith("/")) {
            return baseUrl + path;
        }
        return baseUrl + "/" + path;
    }

    private static String deriveBaseUrl(String url) {
        if (url == null) {
            return "";
        }
        if (url.endsWith("/results")) {
            return url.substring(0, url.length() - "/results".length());
        }
        return url;
    }
}
