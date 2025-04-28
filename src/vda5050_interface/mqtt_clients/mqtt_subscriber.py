from paho.mqtt import client as mqtt_client

class MQTTSubscriber:
    """
    Class for subscribing to MQTT topics.
    """
    def __init__(self, config_data, logging, on_message, channel, client_id) -> None:
        """
        Initialize the MQTT subscriber.

        :param config_data: Data from the configuration file.
        :param logging: Logging object.
        :param on_message: Callback function for the MQTT message.
        :param channel: MQTT channel.
        :param client_id: MQTT client ID.
        """
        self.logging = logging
        self.broker = config_data['mqtt_broker_ip']
        self.port = config_data['mqtt_broker_port']
        self.topic = channel
        self.client_id = client_id
        self.client = mqtt_client.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = on_message
        self.client.connect(self.broker, self.port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc) -> None:
        """
        Callback function for the MQTT client connection.

        :param client: MQTT client.
        :param userdata: User data.
        :param flags: Flags.
        :param rc: Return code.
        """
        if rc == 0:
            self.logging.info(f"Connected client {self.client_id} to MQTT broker.")
            self.client.subscribe(self.topic)
        else:
            self.logging.error(f"Failed to connect client {self.client_id} to MQTT broker, return code {rc}.")
