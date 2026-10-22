# MavlinkEndpoint

`mavlink_endpoint` is a CMake library that provides a MAVLink endpoint on top of a byte-oriented transport. It consumes a stream of raw bytes, frames incoming MAVLink messages, and exposes a simple listener/transmit API so that consumers never have to deal with MAVLink framing or transport details directly.

## Description

`MavlinkEndpoint` is an abstraction that sits between your application and a physical (or virtual) byte transport. It:

- Spawns a background listening thread that reads raw bytes from the underlying transport.
- Frames and parses those bytes into MAVLink messages using the generated C MAVLink headers (`external/mavlink`, a git submodule of `mavlink/c_library_v2`).
- Delivers each fully received message to every registered listener callback as a `MavlinkEndpointPacket` (a `mavlink_message_t` plus the associated `mavlink_status_t`).
- Lets you transmit an already-encoded `mavlink_message_t` via `writeMessage()`.
- Tracks connection state (`DISCONNECTED`, `CONNECTING`, `CONNECTED`, `DISCONNECTING`) and exposes listener management through RAII tokens.

Because MAVLink framing and transport are decoupled, the same endpoint code works with any serial, network, or simulated link.

## Dependencies

- **byte_transport** — an abstraction for byte-oriented transports. `mavlink_endpoint` depends on **`PendarlabByteTransport` version `1.0.0`** (see `BYTE_TRANSPORT_VERSION` in `CMakeLists.txt`).
- **GTest** — required to build the test runner (`mavlink_endpoint_test`).
- **external/mavlink** — git submodule of the MAVLink C library v2. Generated headers are used directly; after cloning, run `git submodule update --init`.

## How to use

### Build

`mavlink_endpoint` locates `PendarlabByteTransport` via `find_package`, so you must point CMake at a prefix where that package is installed using `CMAKE_PREFIX_PATH`. Adjust the path to wherever your `byte_transport` install lives.

```bash
# clone submodules first
git submodule update --init

# configure
cmake -S mavlink_endpoint -B build -G Ninja \
  -DCMAKE_PREFIX_PATH=/path/to/byte_transport/1.0.0

# build
cmake --build build
```

### Run the tests

The test target additionally adds `external/mavlink/common` to its include path.

```bash
./build/mavlink_endpoint_test                        # all tests
./build/mavlink_endpoint_test --gtest_filter='<TestSuite>.<Case>'   # single test
```

### Install

```bash
cmake --install build
```

The library installs as the CMake package `PendarlabMavlinkEndpoint`, exporting the target `pendarlab::MavlinkEndpoint`. The default install prefix is `/usr/local`; override it with `--prefix <dir>` (and keep that prefix on `CMAKE_PREFIX_PATH` when consuming the package).

### Consume from another project

```cmake
find_package(PendarlabMavlinkEndpoint CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE pendarlab::MavlinkEndpoint)
```

## Providing a byte_transport implementation

`byte_transport` is only an abstraction — it defines the `Transport` and `TransportDefinition` interfaces but ships no concrete implementation. `MavlinkEndpoint` does not construct transports directly; instead you must:

1. **Implement** `byte_transport::Transport` (the `read`/`write` interface) for your actual link, along with a `byte_transport::TransportDefinition` that can `parseConfig()` a config map and `create()` the transport.
2. **Register** your `TransportDefinition` in a `byte_transport::Registry` under a string `type`.
3. **Pass** that registry to `MavlinkEndpoint::create(const byte_transport::RegistryUserAccess&)`.

Then, to establish a connection, call `connect(type, cfg)`, which looks up your registered definition, parses the config, and instantiates the transport.

For a concrete example of how to implement and register a `Transport`/`TransportDefinition`, and how to wire it into a `MavlinkEndpoint`, see the test in `test/mavlink_endpoint_test.cpp` (the `MockByteTransport` / `MockByteTransportDefinition` classes and the `MavlinkEndpointTestSetup` fixture).

## API overview

Namespace: `pendarlab::lib::comm`

- `MavlinkEndpoint::create(registry)` — factory returning a `shared_ptr<MavlinkEndpoint>`.
- `createListener(cb)` — register a listener callback; returns a `MavlinkEndpointToken` (RAII — destroying or calling `release()` on the token unregisters the callback).
- `writeMessage(msg)` — encode and transmit a `mavlink_message_t`; returns bytes written, or `-1` if not connected.
- `connect(type, cfg)` / `disconnect()` — manage the transport lifecycle.
- `getState()`, `getListenersID()`, `getNumOfListener()` — introspection helpers.
