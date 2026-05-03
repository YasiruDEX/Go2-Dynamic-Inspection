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
	robotID   = "robot_01"
	Client    mqtt.Client

	// Global State (Thread-Safe)
	StateMutex sync.RWMutex
	PointsMsg  []byte
	TfsMsg     map[string]models.TF
	PathMsg    []models.PathPoint
	VideoMsg   string

	// PoseMsg holds the latest robot pose published by MOLA
	PoseMsg []byte

	MqttCount int
	MqttRate  float64

	// RobotStatusSubs receives inbound robot/{robot_id}/status/# messages for WebSocket broadcast
	RobotStatusSubs   []chan []byte
	robotStatusSubsMu sync.Mutex

	// PoseSubs receives pose messages for /ws/pose WebSocket broadcast
	PoseSubs   []chan []byte
	poseSubsMu sync.Mutex
)

// AddRobotStatusSub registers a channel to receive status messages
func AddRobotStatusSub(ch chan []byte) {
	robotStatusSubsMu.Lock()
	defer robotStatusSubsMu.Unlock()
	RobotStatusSubs = append(RobotStatusSubs, ch)
}

// RemoveRobotStatusSub unregisters a channel
func RemoveRobotStatusSub(ch chan []byte) {
	robotStatusSubsMu.Lock()
	defer robotStatusSubsMu.Unlock()
	updated := RobotStatusSubs[:0]
	for _, c := range RobotStatusSubs {
		if c != ch {
			updated = append(updated, c)
		}
	}
	RobotStatusSubs = updated
}

// broadcastStatus fans out a status payload to all registered WebSocket subscribers
func broadcastStatus(payload []byte) {
	robotStatusSubsMu.Lock()
	defer robotStatusSubsMu.Unlock()
	for _, ch := range RobotStatusSubs {
		select {
		case ch <- payload:
		default:
			// Non-blocking: drop if subscriber is slow
		}
	}
}

// AddPoseSub registers a channel to receive MOLA pose messages
func AddPoseSub(ch chan []byte) {
	poseSubsMu.Lock()
	defer poseSubsMu.Unlock()
	PoseSubs = append(PoseSubs, ch)
}

// RemovePoseSub unregisters a pose channel
func RemovePoseSub(ch chan []byte) {
	poseSubsMu.Lock()
	defer poseSubsMu.Unlock()
	updated := PoseSubs[:0]
	for _, c := range PoseSubs {
		if c != ch {
			updated = append(updated, c)
		}
	}
	PoseSubs = updated
}

// broadcastPose fans out a pose payload to all /ws/pose subscribers
func broadcastPose(payload []byte) {
	poseSubsMu.Lock()
	defer poseSubsMu.Unlock()
	for _, ch := range PoseSubs {
		select {
		case ch <- payload:
		default:
		}
	}
}

// InitMQTT connects to the broker and subscribes to telemetry and robot status topics
func InitMQTT() {
	// Robot ID from env
	if id := os.Getenv("ROBOT_ID"); id != "" {
		robotID = id
	}

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
	opts.SetClientID("go2_mission_planner_backend")
	opts.SetKeepAlive(60 * time.Second)
	opts.SetAutoReconnect(true)

	opts.OnConnect = func(c mqtt.Client) {
		log.Printf("MQTT Connected to %s:%d (robot_id=%s)", broker, port, robotID)

		// Subscribe to legacy telemetry (points, tf, path, video)
		telemetryTopic := "robo_gen_labs/go2_robot_1/telemetry/#"
		if t := c.Subscribe(telemetryTopic, 0, telemetryHandler); t.Wait() && t.Error() != nil {
			log.Printf("Error subscribing to %s: %v", telemetryTopic, t.Error())
		} else {
			log.Printf("Subscribed to %s", telemetryTopic)
		}

		// Subscribe to canonical robot status topics (inbound from robot-bt and MOLA bridge)
		statusTopic := "robot/" + robotID + "/status/#"
		if t := c.Subscribe(statusTopic, 0, statusHandler); t.Wait() && t.Error() != nil {
			log.Printf("Error subscribing to %s: %v", statusTopic, t.Error())
		} else {
			log.Printf("Subscribed to %s", statusTopic)
		}

		// Subscribe to MOLA localmap points (fed by ros2_mqtt_bridge)
		localmapTopic := "robo_gen_labs/go2_robot_1/telemetry/points"
		if t := c.Subscribe(localmapTopic, 0, localmapHandler); t.Wait() && t.Error() != nil {
			log.Printf("Error subscribing to %s: %v", localmapTopic, t.Error())
		} else {
			log.Printf("Subscribed to %s", localmapTopic)
		}

		// Start MQTT rate tracker
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

// telemetryHandler processes legacy robo_gen_labs telemetry messages
func telemetryHandler(client mqtt.Client, msg mqtt.Message) {
	topic := msg.Topic()
	StateMutex.Lock()
	MqttCount++
	defer StateMutex.Unlock()

	const legacyBase = "robo_gen_labs/go2_robot_1"
	switch topic {
	case legacyBase + "/telemetry/points":
		PointsMsg = msg.Payload()
	case legacyBase + "/telemetry/tf":
		var data struct {
			TFs map[string]models.TF `json:"tfs"`
		}
		if err := json.Unmarshal(msg.Payload(), &data); err == nil {
			TfsMsg = data.TFs
		}
	case legacyBase + "/telemetry/path":
		var data struct {
			Path []models.PathPoint `json:"path"`
		}
		if err := json.Unmarshal(msg.Payload(), &data); err == nil {
			PathMsg = data.Path
		}
	case legacyBase + "/telemetry/video":
		VideoMsg = string(msg.Payload())
	}
}

// localmapHandler stores MOLA localmap binary points (already in PointsMsg slot used by /ws/points)
func localmapHandler(client mqtt.Client, msg mqtt.Message) {
	StateMutex.Lock()
	MqttCount++
	PointsMsg = msg.Payload()
	StateMutex.Unlock()
}

// statusHandler processes canonical robot/{robot_id}/status/# messages and broadcasts to WebSocket subs
func statusHandler(client mqtt.Client, msg mqtt.Message) {
	StateMutex.Lock()
	MqttCount++
	StateMutex.Unlock()

	// If this is a pose message, store it and broadcast to /ws/pose subscribers
	poseTopic := "robot/" + robotID + "/status/pose"
	if msg.Topic() == poseTopic {
		StateMutex.Lock()
		PoseMsg = msg.Payload()
		StateMutex.Unlock()
		broadcastPose(msg.Payload())
		return
	}

	// Wrap with topic metadata for WebSocket consumers (non-pose status messages)
	envelope := map[string]interface{}{
		"topic":     msg.Topic(),
		"payload":   json.RawMessage(msg.Payload()),
		"timestamp": time.Now().UTC().Format(time.RFC3339),
	}
	data, err := json.Marshal(envelope)
	if err != nil {
		log.Printf("MQTT status marshal error: %v", err)
		return
	}
	broadcastStatus(data)
}

// cmdTopic returns the canonical command topic for a given command path
func cmdTopic(cmd string) string {
	return "robot/" + robotID + "/cmd/" + cmd
}

// PublishNavigate sends navigation goals to the robot (legacy topic kept for compatibility)
func PublishNavigate(req models.GoalRequest) error {
	payload, err := json.Marshal(req)
	if err != nil {
		return err
	}
	topic := "robo_gen_labs/go2_robot_1/commands/navigate"
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}

// PublishMissionLaunch sends the canonical mission launch command
// topic: robot/{robot_id}/cmd/mission/launch
// payload: {"mission_id": "<MisXXXX>"}
func PublishMissionLaunch(missionMqttID string) error {
	payload, err := json.Marshal(map[string]string{
		"mission_id": missionMqttID,
	})
	if err != nil {
		return err
	}
	topic := cmdTopic("mission/launch")
	log.Printf("MQTT Publish -> %s: %s", topic, string(payload))
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}

// PublishMissionAbort sends the canonical mission abort command
// topic: robot/{robot_id}/cmd/mission/abort
// payload: {}
func PublishMissionAbort() error {
	topic := cmdTopic("mission/abort")
	log.Printf("MQTT Publish -> %s: {}", topic)
	token := Client.Publish(topic, 1, false, []byte("{}"))
	token.Wait()
	return token.Error()
}

// PublishMappingStart sends the canonical mapping start command
// topic: robot/{robot_id}/cmd/mapping/start
// payload: {}
func PublishMappingStart() error {
	topic := cmdTopic("mapping/start")
	log.Printf("MQTT Publish -> %s: {}", topic)
	token := Client.Publish(topic, 1, false, []byte("{}"))
	token.Wait()
	return token.Error()
}

// PublishMappingEnd sends the canonical mapping end command
// topic: robot/{robot_id}/cmd/mapping/end
// payload: {}
func PublishMappingEnd() error {
	topic := cmdTopic("mapping/end")
	log.Printf("MQTT Publish -> %s: {}", topic)
	token := Client.Publish(topic, 1, false, []byte("{}"))
	token.Wait()
	return token.Error()
}

// PublishMappingAbort sends the canonical mapping abort command
// topic: robot/{robot_id}/cmd/mapping/abort
// payload: {}
func PublishMappingAbort() error {
	topic := cmdTopic("mapping/abort")
	log.Printf("MQTT Publish -> %s: {}", topic)
	token := Client.Publish(topic, 1, false, []byte("{}"))
	token.Wait()
	return token.Error()
}

// PublishMappingGenerate sends the canonical map generation command
// topic: robot/{robot_id}/cmd/mapping/generate
// payload: {"session_id": null} or {"session_id": "<uuid>"}
func PublishMappingGenerate(sessionID *string) error {
	payload, err := json.Marshal(map[string]interface{}{
		"session_id": sessionID,
	})
	if err != nil {
		return err
	}
	topic := cmdTopic("mapping/generate")
	log.Printf("MQTT Publish -> %s: %s", topic, string(payload))
	token := Client.Publish(topic, 1, false, payload)
	token.Wait()
	return token.Error()
}

// PublishEStop sends the emergency stop command
// topic: robot/{robot_id}/cmd/system/estop
// payload: {}
func PublishEStop() error {
	topic := cmdTopic("system/estop")
	log.Printf("MQTT Publish -> %s: {}", topic)
	token := Client.Publish(topic, 1, false, []byte("{}"))
	token.Wait()
	return token.Error()
}

// PublishMissionCommand is kept for backward compatibility — routes to the correct canonical function
func PublishMissionCommand(action string, missionMqttID string) error {
	switch action {
	case "start":
		return PublishMissionLaunch(missionMqttID)
	case "terminate", "abort":
		return PublishMissionAbort()
	default:
		return PublishMissionLaunch(missionMqttID)
	}
}

// PublishMappingCommand is kept for backward compatibility — routes to canonical functions
func PublishMappingCommand(action string) error {
	switch action {
	case "start":
		return PublishMappingStart()
	case "stop", "end":
		return PublishMappingEnd()
	case "abort":
		return PublishMappingAbort()
	default:
		return PublishMappingStart()
	}
}
