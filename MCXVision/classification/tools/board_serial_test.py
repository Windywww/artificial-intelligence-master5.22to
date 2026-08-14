#!/usr/bin/env python3
"""Exercise the MCXVision classification binary UART protocol."""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Iterable


BAUD_RATE = 115200
FLAGS = {"goal": 0xFE, "box": 0xBB}


@dataclass(frozen=True)
class RequestResult:
    index: int
    flag_name: str
    response: int
    latency_ms: float


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return parsed


def nonnegative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must not be negative")
    return parsed


def make_packet(flag: int) -> bytes:
    return bytes((0xA5, 0x5A, flag, flag))


def describe_response(value: int) -> str:
    if value == 0:
        return "empty"
    if 1 <= value <= 10:
        return f"label={value - 1}"
    if value == 11:
        return "unknown"
    return "reserved"


def percentile(values: Iterable[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("at least one value is required")
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def open_serial(port: str, timeout_ms: int):
    try:
        import serial
    except ImportError as error:
        raise SystemExit(
            "pyserial is required; install it with: python -m pip install pyserial"
        ) from error

    try:
        connection = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_ms / 1000.0,
            write_timeout=timeout_ms / 1000.0,
        )
    except serial.SerialException as error:
        raise SystemExit(f"cannot open {port}: {error}") from error

    connection.reset_input_buffer()
    connection.reset_output_buffer()
    return connection


def request_once(connection, index: int, flag_name: str) -> RequestResult:
    packet = make_packet(FLAGS[flag_name])
    started_ns = time.perf_counter_ns()
    connection.write(packet)
    connection.flush()
    data = connection.read(1)
    latency_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
    if len(data) != 1:
        raise TimeoutError(
            f"request {index} ({flag_name}) timed out after {latency_ms:.1f} ms"
        )
    return RequestResult(index, flag_name, data[0], latency_ms)


def print_result(result: RequestResult) -> None:
    print(
        f"request={result.index} flag={result.flag_name} "
        f"response={result.response} ({describe_response(result.response)}) "
        f"latency_ms={result.latency_ms:.3f}"
    )


def print_summary(results: list[RequestResult], mode: str) -> None:
    latencies = [result.latency_ms for result in results]
    print(
        f"summary mode={mode} requests={len(results)} "
        f"p50_ms={statistics.median(latencies):.3f} "
        f"p95_ms={percentile(latencies, 0.95):.3f} "
        f"max_ms={max(latencies):.3f}"
    )
    if len(results) > 1:
        steady = latencies[1:]
        phase = "same_model" if mode == "request" else "model_switch"
        print(
            f"summary phase={phase} requests={len(steady)} "
            f"p50_ms={statistics.median(steady):.3f} "
            f"p95_ms={percentile(steady, 0.95):.3f}"
        )


def run_requests(connection, flag_name: str, count: int, interval_ms: int) -> None:
    results = []
    for index in range(1, count + 1):
        result = request_once(connection, index, flag_name)
        results.append(result)
        print_result(result)
        if interval_ms and index != count:
            time.sleep(interval_ms / 1000.0)
    print_summary(results, "request")


def run_stress(connection, requests: int, interval_ms: int) -> None:
    results = []
    flag_names = ("box", "goal")
    for index in range(1, requests + 1):
        result = request_once(connection, index, flag_names[(index - 1) % 2])
        results.append(result)
        print_result(result)
        if interval_ms and index != requests:
            time.sleep(interval_ms / 1000.0)
    print_summary(results, "stress")


def expect_no_response(connection, name: str, packet: bytes) -> None:
    connection.write(packet)
    connection.flush()
    data = connection.read(1)
    if data:
        raise RuntimeError(
            f"{name} packet unexpectedly returned {data[0]} "
            f"({describe_response(data[0])})"
        )
    print(f"protocol={name} response=none status=pass")


def run_protocol(connection, flag_name: str) -> None:
    flag = FLAGS[flag_name]
    expect_no_response(connection, "bad_header", bytes((0x00, 0x5A, flag, flag)))
    expect_no_response(
        connection, "bad_repeated_flag", bytes((0xA5, 0x5A, flag, flag ^ 0x01))
    )
    result = request_once(connection, 1, flag_name)
    print_result(result)
    print("protocol=recovery status=pass")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, for example COM7")
    parser.add_argument("--timeout-ms", type=positive_int, default=2000)
    parser.add_argument("--interval-ms", type=nonnegative_int, default=0)
    commands = parser.add_subparsers(dest="command", required=True)

    request = commands.add_parser("request", help="send one flag repeatedly")
    request.add_argument("--flag", choices=FLAGS, required=True)
    request.add_argument("--count", type=positive_int, default=1)

    stress = commands.add_parser("stress", help="alternate box and goal requests")
    stress.add_argument("--requests", type=positive_int, default=200)

    protocol = commands.add_parser("protocol", help="test rejection and recovery")
    protocol.add_argument("--flag", choices=FLAGS, default="box")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        with open_serial(args.port, args.timeout_ms) as connection:
            if args.command == "request":
                run_requests(connection, args.flag, args.count, args.interval_ms)
            elif args.command == "stress":
                run_stress(connection, args.requests, args.interval_ms)
            else:
                run_protocol(connection, args.flag)
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
