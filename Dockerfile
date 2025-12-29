# Build stage
FROM ubuntu:22.04 AS builder

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Crow (header-only library)
RUN git clone --depth 1 https://github.com/CrowCpp/Crow.git /opt/crow

WORKDIR /app

# Copy source files
COPY CMakeLists.txt main.cpp ./

# Build the application
RUN mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCROW_INCLUDE_DIR=/opt/crow/include \
          .. && \
    cmake --build . --parallel

# Runtime stage - minimal image
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libstdc++6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy built binary from builder stage
COPY --from=builder /app/build/PostAI .

# Render sets PORT environment variable
ENV PORT=10000
EXPOSE 10000

# Run the server
CMD ["./PostAI"]
