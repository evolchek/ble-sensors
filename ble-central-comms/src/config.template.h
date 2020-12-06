const char ssid[] = "...";
const char pass[] = "...";

#define THINGNAME "..."

const char caCertStr[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char clientCertStr[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
...
-----END CERTIFICATE-----
)EOF";

const char privKeyStr[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
...
-----END RSA PRIVATE KEY-----
)EOF";

const char mqttHost[] = "XXXXXX.iot.us-east-1.amazonaws.com";
const _ushort mqttPort = 8883;
const char mqttSubTopic[] = "$aws/things/" THINGNAME "/shadow/update";
const char mqttPubTopic[] = "$aws/things/" THINGNAME "/shadow/update";

const _uchar i2cCommsAddress = 1;