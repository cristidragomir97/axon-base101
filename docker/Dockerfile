# Zenoh router with SERIAL transport for the Axon board.
#
# The stock `eclipse/zenoh` image is built without the (non-default)
# `transport_serial` Cargo feature, so it cannot speak to the Axon's
# zenoh-pico serial link over USB CDC. This image rebuilds `zenohd` with that
# feature and overlays it onto the official image (keeping its runtime base).
#
# IMPORTANT: keep the router protocol-compatible with the firmware's vendored
# zenoh-pico. That is currently 1.9.0 (see lib/zenoh-pico/include/zenoh-pico.h,
# `#define ZENOH_PICO`). Override with --build-arg ZENOH_REF=<tag> if you bump
# zenoh-pico.
ARG ZENOH_REF=1.9.0

# --- builder: compile zenohd with transport_serial -------------------------
FROM rust:bookworm AS builder
ARG ZENOH_REF
RUN apt-get update && apt-get install -y --no-install-recommends \
        git clang libclang-dev && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch "${ZENOH_REF}" https://github.com/eclipse-zenoh/zenoh /src
WORKDIR /src
RUN cargo build --release -p zenohd --features transport_serial \
 && cp target/release/zenohd /zenohd

# --- runtime: official image + the serial-enabled binary -------------------
FROM eclipse/zenoh:latest
COPY --from=builder /zenohd /usr/local/bin/zenohd
COPY zenoh-serial.json5 /etc/zenoh/serial.json5

# Override the stock entrypoint so the path is deterministic regardless of
# where the base image placed its own zenohd.
ENTRYPOINT ["/usr/local/bin/zenohd"]
CMD ["-c", "/etc/zenoh/serial.json5"]
