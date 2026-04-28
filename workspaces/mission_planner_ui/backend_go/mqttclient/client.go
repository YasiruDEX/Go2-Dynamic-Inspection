package mqttclient

import (
	"encoding/json"
	"log"
	"os"
	"strconv"
	"sync"
	"time"

	"mission_planner_backend/models"

	mqtt "github.com/eclipse/paho.mqtt.golang"
)

var (
	baseTopic = "robo_gen_labs/go2_robot_1"
	Client    mqtt.Client

	// Global State (Thread-Safe)
	StateMutex sync.RWMutex
	PointsMsg  []byte
	TfsMsg     map[string]models.TF
	PathMsg    []models.PathPoint
	VideoMsg   string

	MqttCount int
	MqttRate  float64
)

// InitMQTT connects to the broker and subscribes
func InitMQTT() {
	broker := os.Getenv("MQTT_BROKER")
	if broker == "" {
		broker = "broker.hivemq.com"
	}

	portStr := os.Getenv("MQTT_PORT")
	port := 1883
	if portStr != "" {
		if p, err := strconv.Atoi(portStr); err == nil {
			port = p
		}
	}

	opts := mqtt.NewClientOptions()
	opts.AddBroker("tcp://" + broker + ":" + strconv.Itoa(port))
	opts.SetClientID("go2_fastapi_backend_go_migration")

	opts.OnConnect = func(c mqtt.Client) {
		log.Printf("MQTT Connected to %s:%d", broker, port)
		topic := baseTopic + "/telemetry/#"
		token := c.Subscribe(topic, 0, messageHandler)

		go func() {
			if token.Wait() && token.Error() != nil {
				log.Printf("Error subscribing to %s: %v", topic, token.Error())
			} else {
				log.Printf("Subscribed to %s", topic)
			}
		}()

		// Start a background goroutine to calculate the message rate every second
		go func() {
			ticker := time.NewTicker(1 * time.Second)
			defer ticker.Stop()
			for range ticker.C {
				StateMutex.Lock()
				MqttRate = float64(MqttCount)
				MqttCount = 0
				StateMutex.Unlock()
			}
		}()
	}

	opts.OnConnectionLost = func(c mqtt.Client, err error) {
		log.Printf("MQTT Connection lost: %v", err)
	}

	Client = mqtt.NewClient(opts)
	if token := Client.Connect(); token.Wait() && token.Error() != nil {
		log.Printf("MQTT Connection Error: %v", token.Error())
	}
}

// messageHandler processes incoming telemetry
func messageHandler(client mqtt.Client, msg mqtt.Message) {
	topic := msg.Topic()
	StateMutex.Lock()
	MqttCount++
	defer StateMutex.Unlock()

	// Topic dispatch
	if topic == baseTopic+"/telemetry/points" {
		PointsMsg = msg.Payload()
	} else if topic == baseTopic+"/telemetry/tf" {
		var data struct {
			TFs map[string]models.TF `json:"tfs"`
		}
		if err := json.Unmarshal(msg.Payload(), &data); err == nil {
			TfsMsg = data.TFs
		}
	} else if topic == baseTopic+"/telemetry/path" {
		var data struct {
			Path []models.PathPoint `json:"path"`
		}
		if err := json.Unmarshal(msg.Payload(), &data); err == nil {
			PathMsg = data.Path
		}
	} else if topic == baseTopic+"/telemetry/video" {
		VideoMsg = string(msg.Payload())
	}
}

// PublishNavigate sends navigation goals to the robot
func PublishNavigate(req models.GoalRequest) error {
	payload, err := json.Marshal(req)
	if err != nil {
		return err
	}
	topic := baseTopic + "/commands/navigate"
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}

// PublishMissionCommand sends mission start/terminate commands via MQTT
func PublishMissionCommand(action string, missionName string) error {
	payload, err := json.Marshal(map[string]string{
		"action":  action,
		"mission": missionName,
	})
	if err != nil {
		return err
	}
	topic := baseTopic + "/mission"
	log.Printf("MQTT Publish -> %s: %s", topic, string(payload))
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}

// PublishMappingCommand sends mapping start/stop commands via MQTT
func PublishMappingCommand(action string) error {
	payload, err := json.Marshal(map[string]string{
		"action": action,
	})
	if err != nil {
		return err
	}
	topic := baseTopic + "/mapping"
	log.Printf("MQTT Publish -> %s: %s", topic, string(payload))
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}
