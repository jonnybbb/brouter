# =============================================================================
# BRouter Server — Multi-arch Docker image (amd64 + arm64)
# Builds BRouter from source using Gradle, runs the standalone HTTP server.
# Uses Bellsoft Liberica to share base layers with the Spring Boot app image.
# =============================================================================
# glibc + JDK 17: the daemon JVM is pinned to 17 (gradle-daemon-jvm.properties)
# and Gradle 9's native services do not run on musl, so an Alpine build stage
# cannot work on either axis. The runtime stage below stays Alpine.
FROM bellsoft/liberica-openjdk-debian:17@sha256:9a38411cf122f56caed5b6e28462a0cdf1d4525aa904544028ffb4418ba691ad AS build

WORKDIR /tmp/brouter
COPY . .
RUN chmod +x gradlew && \
    ./gradlew clean fatJar -x test --no-daemon

# --- Runtime stage ---
FROM bellsoft/liberica-openjre-alpine:25@sha256:87025d11840c8e873019b59f2d64a6b3da4bc5e126bb6d51aa3cd86f1b8b27be

RUN addgroup -S brouter && adduser -S brouter -G brouter

WORKDIR /brouter

# Copy the fat JAR
COPY --from=build /tmp/brouter/brouter-server/build/libs/brouter-*-all.jar /brouter/brouter-server.jar

# Copy default profiles (lookups.dat + *.brf)
COPY --from=build /tmp/brouter/misc/profiles2 /brouter/profiles2

# Segments directory — mount your .rd5 files here
# customprofiles is required as 3rd arg to RouteServer (user-defined profiles)
RUN mkdir -p /brouter/segments4 /brouter/customprofiles && chown -R brouter:brouter /brouter

VOLUME ["/brouter/segments4"]

# Runtime configuration — override via environment variables
ENV JAVA_OPTS="-Xmx512m -Xms256m" \
    BROUTER_PORT=17777 \
    BROUTER_MAX_THREADS=4

USER brouter

EXPOSE 17777

# Shell form so env vars are expanded at runtime
ENTRYPOINT exec java $JAVA_OPTS \
    -cp /brouter/brouter-server.jar \
    btools.server.RouteServer \
    /brouter/segments4 /brouter/profiles2 customprofiles \
    $BROUTER_PORT $BROUTER_MAX_THREADS

