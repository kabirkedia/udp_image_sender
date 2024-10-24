import time

class BandwidthManager:
    def __init__(self, max_bandwidth_bps):
        self.max_bandwidth_bps = max_bandwidth_bps  # bits per second
        self.reset_time = time.time()
        self.bytes_sent = 0

    def can_send(self, bytes_to_send):
        current_time = time.time()
        elapsed = current_time - self.reset_time
        allowed_bytes = (self.max_bandwidth_bps * elapsed) / 8  # Convert bits to bytes
        if self.bytes_sent + bytes_to_send <= allowed_bytes:
            self.bytes_sent += bytes_to_send
            return True
        else:
            # Need to wait
            return False

    def wait_until_can_send(self, bytes_to_send):
        while not self.can_send(bytes_to_send):
            time.sleep(0.01)  # Sleep for 10 ms
