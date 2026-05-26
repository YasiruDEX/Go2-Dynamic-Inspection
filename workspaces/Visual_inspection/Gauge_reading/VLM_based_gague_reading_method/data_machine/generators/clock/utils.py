from typing import Union, Tuple
from datetime import datetime, timedelta


def parse_time(t: Union[str, Tuple[int, int, int], datetime]) -> Tuple[int, int, int]:
    if isinstance(t, tuple) and len(t) == 3:
        h, m, s = t
        return int(h), int(m), int(s)
    if isinstance(t, datetime):
        return t.hour, t.minute, t.second
    if isinstance(t, str):
        parts = t.strip().split(":")
        if len(parts) == 2:
            h, m = parts
            s = 0
        elif len(parts) == 3:
            h, m, s = parts
        else:
            raise ValueError("Time string must be 'HH:MM' or 'HH:MM:SS'")
        return int(h), int(m), int(s)
    raise TypeError("Unsupported time input. Use 'HH:MM[:SS]', (h,m,s), or datetime.")


def time_to_string(hours, minutes, seconds=0, with_seconds=True):
    """Convert time components back to string format"""
    if with_seconds:
        return f"{hours}:{minutes:02d}:{seconds:02d}"
    else:
        return f"{hours}:{minutes:02d}"


def add_seconds_to_time_string(time_str, seconds_delta):
    """Add seconds to a time string and return the new time string"""
    h, m, s = parse_time(time_str)
    # Create a datetime object for easy arithmetic
    dt = datetime.now().replace(hour=h, minute=m, second=s)
    new_dt = dt + timedelta(seconds=seconds_delta)
    return time_to_string(new_dt.hour, new_dt.minute, new_dt.second)


def add_minutes_to_time_string(time_str, minutes_delta):
    """Add minutes to a time string and return the new time string"""
    h, m, s = parse_time(time_str)
    dt = datetime.now().replace(hour=h, minute=m)
    new_dt = dt + timedelta(minutes=minutes_delta)
    return time_to_string(new_dt.hour, new_dt.minute, new_dt.second, with_seconds=False)
