package btools.mapcreator;

import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * {@link PmTilesArchive.ByteSource} over a remote archive addressed with HTTP range
 * requests; Mapterhorn serves the planet archive behind a CDN that honours
 * {@code accept-ranges: bytes}.
 * <p>
 * The first successful range response establishes a strong ETag and total content
 * length; later requests send {@code If-Match} and must return the same values, so one
 * conversion cannot silently mix two remote archive versions. Transient failures
 * (5xx, 408, 429) are retried with jittered exponential backoff, honouring
 * {@code Retry-After} on 429/503 in both delta-seconds and HTTP-date form, capped so
 * one tile cannot stall a build.
 */
public final class HttpRangeByteSource implements PmTilesArchive.ByteSource {
  private static final int MAX_ATTEMPTS = 3;
  private static final int TIMEOUT_MS = 30000;
  private static final Pattern CONTENT_RANGE =
    Pattern.compile("bytes (\\d+)-(\\d+)/(\\d+)");

  private final String url;
  private final Object snapshotLock = new Object();
  private String etag;
  private long sourceLength = -1L;
  private String versionId;

  public HttpRangeByteSource(String url) {
    this.url = url;
  }

  @Override
  public byte[] read(long offset, int length) throws IOException {
    long end = PmTilesArchive.checkedReadEnd(offset, length) - 1L;
    IOException last = null;
    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
      try {
        return rangeGet(offset, end, length);
      } catch (PermanentHttpException e) {
        throw e; // a 4xx or snapshot mismatch will not improve on retry
      } catch (RateLimitedException e) {
        last = e;
        if (attempt + 1 >= MAX_ATTEMPTS) {
          break;
        }
        // honour the server's Retry-After, capped so one tile cannot stall a build
        sleepBeforeRetry(withJitter(
          Math.max(200L << attempt, Math.min(e.retryAfterMs, 10000L))));
      } catch (IOException e) {
        last = e;
        if (attempt + 1 >= MAX_ATTEMPTS) {
          break; // no point sleeping after the final attempt
        }
        sleepBeforeRetry(withJitter(200L << attempt));
      }
    }
    throw new IOException("range read failed at " + offset + " (+" + length + ")", last);
  }

  @Override
  public long size() throws IOException {
    synchronized (snapshotLock) {
      requireSnapshot();
      return sourceLength;
    }
  }

  @Override
  public String versionId() throws IOException {
    synchronized (snapshotLock) {
      requireSnapshot();
      return versionId;
    }
  }

  private void requireSnapshot() throws IOException {
    if (etag == null) {
      throw new IOException("HTTP source metadata is unavailable before a successful read");
    }
  }

  private static void sleepBeforeRetry(long delayMs) throws IOException {
    try {
      Thread.sleep(delayMs);
    } catch (InterruptedException ie) {
      Thread.currentThread().interrupt();
      throw new IOException("interrupted", ie);
    }
  }

  /**
   * Equal jitter, so eight fetch threads retrying the same outage do not hammer the
   * CDN in lockstep.
   */
  private static long withJitter(long delayMs) {
    long half = delayMs / 2;
    return half + java.util.concurrent.ThreadLocalRandom.current().nextLong(half + 1);
  }

  /**
   * Retry-After per RFC 9110: delta-seconds or an HTTP-date. Absent or unparseable
   * values fall back to one second.
   */
  static long parseRetryAfterMs(String value) {
    if (value == null) {
      return 1000L;
    }
    String trimmed = value.trim();
    try {
      return Math.max(0L, Long.parseLong(trimmed)) * 1000L;
    } catch (NumberFormatException notSeconds) {
      try {
        long at = java.time.ZonedDateTime
          .parse(trimmed, java.time.format.DateTimeFormatter.RFC_1123_DATE_TIME)
          .toInstant().toEpochMilli();
        return Math.max(0L, at - System.currentTimeMillis());
      } catch (RuntimeException notDate) {
        return 1000L;
      }
    }
  }

  /** A deterministic HTTP failure that a retry cannot fix. */
  private static final class PermanentHttpException extends IOException {
    PermanentHttpException(String message) {
      super(message);
    }
  }

  /** HTTP 429/503: retryable, carrying the server's requested delay. */
  private static final class RateLimitedException extends IOException {
    final long retryAfterMs;

    RateLimitedException(String message, long retryAfterMs) {
      super(message);
      this.retryAfterMs = retryAfterMs;
    }
  }

  private byte[] rangeGet(long offset, long end, int length) throws IOException {
    String ifMatch;
    synchronized (snapshotLock) {
      ifMatch = etag;
    }
    HttpURLConnection con = (HttpURLConnection) new URL(url).openConnection();
    con.setRequestMethod("GET");
    con.setRequestProperty("Range", "bytes=" + offset + "-" + end);
    con.setRequestProperty("Accept-Encoding", "identity");
    if (ifMatch != null) {
      con.setRequestProperty("If-Match", ifMatch);
    }
    con.setRequestProperty("User-Agent", "BRouter map-creator");
    con.setConnectTimeout(TIMEOUT_MS);
    con.setReadTimeout(TIMEOUT_MS);

    int code = con.getResponseCode();
    if (code != HttpURLConnection.HTTP_PARTIAL) {
      String retryAfter = con.getHeaderField("Retry-After");
      drainAndDisconnect(con);
      String msg = "expected 206 for range request, got " + code + " from " + url;
      if (code == 429 || code == 503) {
        // both carry Retry-After per RFC 9110 and both are worth waiting out
        throw new RateLimitedException(msg, parseRetryAfterMs(retryAfter));
      }
      if (code == 408) {
        throw new IOException(msg); // request timeout: retryable with plain backoff
      }
      if ((code >= 200 && code < 300) || (code >= 400 && code < 500)) {
        throw new PermanentHttpException(msg);
      }
      throw new IOException(msg);
    }

    String contentEncoding = con.getHeaderField("Content-Encoding");
    if (contentEncoding != null && !"identity".equalsIgnoreCase(contentEncoding.trim())) {
      drainAndDisconnect(con);
      throw new PermanentHttpException("unexpected Content-Encoding for range response: "
        + contentEncoding);
    }

    String responseEtag = con.getHeaderField("ETag");
    if (!isStrongEtag(responseEtag)) {
      drainAndDisconnect(con);
      throw new PermanentHttpException("range response requires a strong ETag, got: "
        + responseEtag);
    }

    String contentRange = con.getHeaderField("Content-Range");
    long responseLength;
    Matcher matcher = contentRange == null ? null : CONTENT_RANGE.matcher(contentRange);
    try {
      if (matcher == null || !matcher.matches()
          || Long.parseLong(matcher.group(1)) != offset
          || Long.parseLong(matcher.group(2)) != end) {
        throw new IllegalArgumentException();
      }
      responseLength = Long.parseLong(matcher.group(3));
      if (responseLength <= end) {
        throw new IllegalArgumentException();
      }
    } catch (IllegalArgumentException e) {
      drainAndDisconnect(con);
      throw new PermanentHttpException("unexpected Content-Range for requested bytes " + offset
        + "-" + end + ": " + contentRange);
    }

    checkResponseSnapshot(con, responseEtag, responseLength);
    // Read the exact advertised range and do not call disconnect(), so the connection
    // returns to the keep-alive pool. Callers should also fetch tiles concurrently:
    // range requests still carry network round-trip latency regardless of size.
    byte[] out = new byte[length];
    try (InputStream is = con.getInputStream()) {
      int read = 0;
      while (read < length) {
        int n = is.read(out, read, length - read);
        if (n < 0) {
          throw new IOException("short range read: wanted " + length + ", got " + read);
        }
        read += n;
      }
    }
    captureResponseSnapshot(responseEtag, responseLength);
    return out;
  }

  private void checkResponseSnapshot(HttpURLConnection con, String responseEtag,
      long responseLength) throws PermanentHttpException {
    synchronized (snapshotLock) {
      if (etag == null) {
        return;
      }
      try {
        verifySnapshotLocked(responseEtag, responseLength);
      } catch (PermanentHttpException e) {
        drainAndDisconnect(con);
        throw e;
      }
    }
  }

  private void captureResponseSnapshot(String responseEtag, long responseLength)
      throws PermanentHttpException {
    synchronized (snapshotLock) {
      if (etag == null) {
        etag = responseEtag;
        sourceLength = responseLength;
        versionId = snapshotVersionId(url, responseEtag, responseLength);
        return;
      }
      verifySnapshotLocked(responseEtag, responseLength);
    }
  }

  /** Both snapshot checks share one comparison, so their diagnostics cannot drift. */
  private void verifySnapshotLocked(String responseEtag, long responseLength)
      throws PermanentHttpException {
    if (!etag.equals(responseEtag)) {
      throw new PermanentHttpException("HTTP source ETag changed from " + etag + " to "
        + responseEtag);
    }
    if (sourceLength != responseLength) {
      throw new PermanentHttpException("HTTP source length changed from " + sourceLength
        + " to " + responseLength);
    }
  }

  private static String snapshotVersionId(String url, String etag, long sourceLength) {
    MessageDigest identity = MapterhornFiles.sha256Digest();
    updateLengthPrefixed(identity, url);
    updateLengthPrefixed(identity, etag);
    identity.update(ByteBuffer.allocate(Long.BYTES).order(ByteOrder.BIG_ENDIAN)
      .putLong(sourceLength).array());
    return "http-sha256:" + MapterhornFiles.hex(identity.digest());
  }

  private static void updateLengthPrefixed(MessageDigest identity, String value) {
    byte[] bytes = value.getBytes(StandardCharsets.UTF_8);
    identity.update(ByteBuffer.allocate(Integer.BYTES).order(ByteOrder.BIG_ENDIAN)
      .putInt(bytes.length).array());
    identity.update(bytes);
  }

  private static boolean isStrongEtag(String value) {
    if (value == null || value.length() < 2 || value.startsWith("W/")
        || value.charAt(0) != '"' || value.charAt(value.length() - 1) != '"') {
      return false;
    }
    for (int i = 1; i < value.length() - 1; i++) {
      char c = value.charAt(i);
      if (c == '"' || c <= 0x20 || c == 0x7f) {
        return false;
      }
    }
    return true;
  }

  private static void drainAndDisconnect(HttpURLConnection con) {
    try (InputStream err = con.getErrorStream()) {
      if (err != null) {
        byte[] sink = new byte[4096];
        while (err.read(sink) >= 0) {
          continue;
        }
      }
    } catch (IOException ignored) {
      // best effort only
    }
    con.disconnect();
  }

  @Override
  public void close() {
    // nothing to release
  }
}
