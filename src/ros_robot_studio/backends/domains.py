"""RTPS domain/port validation; domains are routing contexts, never permissions."""

from dataclasses import dataclass
from pathlib import Path

# DDSI-RTPS default port mapping, used by rmw_fastrtps_cpp.
RTPS_BASE = 7400
DOMAIN_GAIN = 250
UNICAST_OFFSET = 11
PARTICIPANT_GAIN = 2
UDP_MAX = 65535
MAX_DOMAIN = (UDP_MAX - RTPS_BASE - UNICAST_OFFSET) // DOMAIN_GAIN


def validate_domain(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= MAX_DOMAIN:
        raise ValueError(
            f"Fast DDS domain must be 0..{MAX_DOMAIN} for the default RTPS port mapping"
        )
    return value


@dataclass(frozen=True)
class DomainDiagnostic:
    severity: str
    message: str


class ROSDomainManager:
    def __init__(self, ephemeral_range=None):
        if ephemeral_range is None:
            try:
                ephemeral_range = tuple(
                    map(int, Path("/proc/sys/net/ipv4/ip_local_port_range").read_text().split())
                )
            except (OSError, ValueError):
                ephemeral_range = None
        self.ephemeral_range = ephemeral_range

    def inspect(self, profile, profiles=(), participants=16):
        domain = validate_domain(profile.domain_id)
        first = RTPS_BASE + DOMAIN_GAIN * domain
        last = first + UNICAST_OFFSET + PARTICIPANT_GAIN * (participants - 1)
        rows = [
            DomainDiagnostic(
                "PASS",
                f"Domain {domain}: DDS UDP ports {first}..{last} for {participants} participants",
            )
        ]
        if last > UDP_MAX:
            rows.append(DomainDiagnostic("FAIL", "Participant count exceeds UDP port range"))
        if (
            self.ephemeral_range
            and first <= self.ephemeral_range[1]
            and last >= self.ephemeral_range[0]
        ):
            rows.append(
                DomainDiagnostic(
                    "WARNING",
                    f"DDS ports overlap this host ephemeral range {self.ephemeral_range}; choose another domain or configure the host explicitly",
                )
            )
        peers = [p.name for p in profiles if p.id != profile.id and p.domain_id == domain]
        if peers:
            rows.append(
                DomainDiagnostic(
                    "WARNING",
                    "Shared domain may expose matching ROS interfaces: " + ", ".join(peers),
                )
            )
            if not profile.shared_domain_confirmed:
                rows.append(
                    DomainDiagnostic(
                        "FAIL", "Explicit shared-domain confirmation required before connecting"
                    )
                )
        return rows
