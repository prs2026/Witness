"""Connection defaults shared by the Witness app and Horizon unified app."""

# The Witness feed is intentionally local and receive-only. No command bridge.
WITNESS_FEED_HOST = "127.0.0.1"
WITNESS_FEED_PORT = 8766

DEFAULT_AIM_MQTT_HOST = "localhost"
DEFAULT_AIM_MQTT_PORT = 1883
DEFAULT_AIM_MQTT_UUID = "124"
DEFAULT_AIM_MQTT_BASE_PATH = "entacore/aim/1234"
