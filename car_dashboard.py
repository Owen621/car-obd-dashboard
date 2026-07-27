"""
Live OBD-II dashboard with an adaptive smoothness score.

Improvements over the first version:
- Thresholds are calculated FROM your own drive's data (mean + std dev)
  instead of arbitrary fixed numbers - adapts to your actual driving/car.
- Tracks "jerk" (rate of change of acceleration) as well as raw
  acceleration - sudden snappy changes get penalised even if the peak
  acceleration itself wasn't extreme.
- Penalty is scaled by HOW severe an event was, not just yes/no.
- Braking events at low speed are weighted down, since that's usually
  just normal stop-start driving rather than harsh braking.
- Final score is normalised per minute of driving, so a short drive and
  a long drive with the same number of harsh events don't score the same.

All scoring is calculated once at the end of the drive, using the full
log - live view still shows raw numbers as you drive.
"""

import obd
import time
import csv
import statistics
from datetime import datetime
from rich.live import Live
from rich.table import Table
from rich.panel import Panel
from rich.console import Console

console = Console()

# --- Configuration ---
DONGLE_ADDRESS = "socket://192.168.0.10:35000"  # check your dongle's manual/box
ENGINE_OFF_READINGS_LIMIT = 6   # consecutive zero-RPM readings before "engine off"
POLL_INTERVAL = 0.5             # seconds between readings

OUTLIER_STD_MULTIPLIER = 2.0    # accel events beyond this many std devs count as "harsh"
JERK_STD_MULTIPLIER = 2.0       # same idea, but for sudden changes in acceleration
LOW_SPEED_DAMPEN_KMH = 15       # braking below this speed is weighted down (stop-start traffic)
LOW_SPEED_BRAKE_WEIGHT = 0.4    # how much to discount low-speed braking events
SCORE_SCALE = 1.5               # tunable - controls how harshly penalty-per-minute hits the score
                                 # (was 5 - too aggressive, floored every real drive to 0)
MAX_PLAUSIBLE_ACCEL = 20        # km/h per second - no real car does more than this; anything
                                 # beyond it is treated as a sensor glitch, not real driving


def connect():
    console.print("[yellow]Connecting to dongle...[/yellow]")
    connection = obd.OBD(DONGLE_ADDRESS)
    if not connection.is_connected():
        console.print("[red]Failed to connect. Check you're on the dongle's WiFi "
                       "and that the IP/port above matches your dongle's manual.[/red]")
        exit()
    console.print("[green]Connected! Starting the engine will begin real readings.[/green]\n")
    return connection


def build_table(speed, rpm, accel, readings_count):
    table = Table(title="Live Drive Data")
    table.add_column("Metric")
    table.add_column("Value")
    table.add_row("Speed (km/h)", f"{speed:.1f}")
    table.add_row("RPM", str(rpm))
    table.add_row("Acceleration (km/h/s)", f"{accel:.2f}")
    table.add_row("Readings logged", str(readings_count))
    table.add_row("Score", "calculated at end of drive")
    return table


def calculate_adaptive_score(readings):
    """Scores the drive using thresholds derived from this drive's own
    acceleration/jerk distribution, rather than fixed guessed numbers."""
    if len(readings) < 5:
        return 100, {}

    accels = [r["accel"] for r in readings[1:]]
    jerks = [r["jerk"] for r in readings[2:]]

    accel_mean = statistics.mean(accels)
    accel_std = statistics.pstdev(accels) or 0.01
    jerk_mean = statistics.mean(jerks) if jerks else 0
    jerk_std = (statistics.pstdev(jerks) if jerks else 0) or 0.01

    accel_threshold = accel_std * OUTLIER_STD_MULTIPLIER
    jerk_threshold = jerk_std * JERK_STD_MULTIPLIER

    total_penalty = 0
    harsh_accel_events = 0
    harsh_brake_events = 0
    harsh_jerk_events = 0

    for r in readings[2:]:
        accel_dev = r["accel"] - accel_mean
        speed = r["speed"]

        if abs(accel_dev) > accel_threshold:
            severity = abs(accel_dev) - accel_threshold
            weight = 1.0
            if r["accel"] < 0 and speed < LOW_SPEED_DAMPEN_KMH:
                weight = LOW_SPEED_BRAKE_WEIGHT
            total_penalty += severity * weight
            if r["accel"] > 0:
                harsh_accel_events += 1
            else:
                harsh_brake_events += 1

        jerk_dev = r["jerk"] - jerk_mean
        if abs(jerk_dev) > jerk_threshold:
            total_penalty += (abs(jerk_dev) - jerk_threshold) * 0.5
            harsh_jerk_events += 1

    trip_duration_minutes = max((readings[-1]["time"] - readings[0]["time"]) / 60, 0.1)
    penalty_per_minute = total_penalty / trip_duration_minutes

    score = max(0, 100 - penalty_per_minute * SCORE_SCALE)

    breakdown = {
        "harsh_accel_events": harsh_accel_events,
        "harsh_brake_events": harsh_brake_events,
        "harsh_jerk_events": harsh_jerk_events,
        "trip_duration_minutes": round(trip_duration_minutes, 1),
        "penalty_per_minute": round(penalty_per_minute, 2),
    }

    return round(score, 1), breakdown


def main():
    connection = connect()

    log_filename = f"drive_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    log_file = open(log_filename, "w", newline="")
    csv_writer = csv.writer(log_file)
    csv_writer.writerow(["timestamp", "speed_kmh", "rpm", "throttle", "acceleration", "jerk"])

    readings = []

    prev_speed = None
    prev_time = None
    prev_accel = None
    engine_off_streak = 0

    with Live(refresh_per_second=2) as live:
        while True:
            speed_response = connection.query(obd.commands.SPEED)
            rpm_response = connection.query(obd.commands.RPM)
            throttle_response = connection.query(obd.commands.THROTTLE_POS)

            current_time = time.time()
            current_speed = speed_response.value.magnitude if speed_response.value is not None else 0
            current_rpm = rpm_response.value.magnitude if rpm_response.value is not None else 0
            current_throttle = throttle_response.value.magnitude if throttle_response.value is not None else 0

            if current_rpm == 0:
                engine_off_streak += 1
            else:
                engine_off_streak = 0

            if engine_off_streak >= ENGINE_OFF_READINGS_LIMIT and len(readings) > 0:
                break

            acceleration = 0
            jerk = 0
            if prev_speed is not None and prev_time is not None:
                time_diff = current_time - prev_time
                if time_diff > 0:
                    raw_accel = (current_speed - prev_speed) / time_diff

                    if abs(raw_accel) > MAX_PLAUSIBLE_ACCEL:
                        # Sensor glitch (e.g. a garbled OBD read) - no real car
                        # accelerates this fast. Discard the bad speed value and
                        # carry the last known good speed forward instead.
                        current_speed = prev_speed
                        acceleration = 0
                    else:
                        acceleration = raw_accel
                        if prev_accel is not None:
                            jerk = (acceleration - prev_accel) / time_diff

            readings.append({
                "time": current_time,
                "speed": current_speed,
                "rpm": current_rpm,
                "throttle": current_throttle,
                "accel": acceleration,
                "jerk": jerk,
            })

            csv_writer.writerow([current_time, current_speed, current_rpm,
                                  current_throttle, acceleration, jerk])

            table = build_table(current_speed, current_rpm, acceleration, len(readings))
            live.update(table)

            prev_speed = current_speed
            prev_time = current_time
            prev_accel = acceleration

            time.sleep(POLL_INTERVAL)

    log_file.close()

    score, breakdown = calculate_adaptive_score(readings)

    console.print("\n")
    if breakdown:
        console.print(Panel(
            f"[bold]Drive complete![/bold]\n\n"
            f"Duration: {breakdown['trip_duration_minutes']} min\n"
            f"Harsh acceleration events: {breakdown['harsh_accel_events']}\n"
            f"Harsh braking events: {breakdown['harsh_brake_events']}\n"
            f"Harsh jerk (sudden change) events: {breakdown['harsh_jerk_events']}\n"
            f"Penalty per minute: {breakdown['penalty_per_minute']}\n\n"
            f"[bold cyan]Final Smoothness Score: {score}/100[/bold cyan]\n\n"
            f"Log saved to: {log_filename}",
            title="Drive Summary",
            border_style="green"
        ))
    else:
        console.print(Panel(
            "Not enough data collected for a reliable score - try a longer drive.",
            title="Drive Summary",
            border_style="red"
        ))


if __name__ == "__main__":
    main()