- esp32 c3 x2
- Arduino
- private server MQTT 2 topic [send data, water on]

[esp32 c3 Node 0 (Root)] # the only one connected to the internet
- Drives a relay to open and close the water, following what the server sends:
  when to open it and for how long, calculated on the server
  * open it now, for as long as the payload says
- Receives data from the outdoor node and sends it up to the server over MQTT

[esp32 c3 Node 1 (Child)]
- dht11 / dht22 measuring humidity and temperature, 3 readings averaged before sending
- Reads and sends the data to Node 0, selectable interval:
  every 1 minute, 5 minutes, 15 minutes, 30 minutes, 1 hour


[TODO - software features to add]
- Node 1 finds Node 0's channel by itself : currently WIFI_CHANNEL is hardcoded,
  so if the router changes channel the link dies until Node 1 is reflashed
- Node 1 finds Node 0's MAC by itself : currently NODE0_MAC is copied by hand from
  Node 0's serial output; Node 1 should broadcast and let Node 0 answer with its MAC
- MQTT QoS : farm/cmd must be subscribed at QoS 1 so a watering command is never
  lost, and farm/data published at QoS 1 so readings survive a broker hiccup
- Server can change the measure/report interval : currently INTERVAL_MIN is a
  #define, so changing it requires reflashing Node 1
