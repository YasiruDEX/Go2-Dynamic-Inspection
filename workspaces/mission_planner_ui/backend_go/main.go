package main

import (
	"log"
	"os"

	"mission_planner_backend/api"
	"mission_planner_backend/database"
	"mission_planner_backend/mqttclient"

	"github.com/gin-contrib/cors"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
)

func main() {
	if err := godotenv.Load(); err != nil {
		if err2 := godotenv.Load("../backend/.env"); err2 != nil {
			log.Println("No .env file found, proceeding with environment variables")
		}
	}

	// Init Singletons
	dbUrl := os.Getenv("DATABASE_URL")
	log.Println("Connecting to Database...")
	database.InitDB(dbUrl)
	log.Println("Database connection initialized.")

	log.Println("Connecting to MQTT Broker...")
	mqttclient.InitMQTT()
	log.Println("MQTT Broker connected.")
	defer mqttclient.Client.Disconnect(250)

	// Router setup
	r := gin.Default()

	// CORS: Allow everything for the local command center
	r.Use(cors.New(cors.Config{
		AllowAllOrigins:  true,
		AllowMethods:     []string{"GET", "POST", "PUT", "PATCH", "DELETE", "OPTIONS"},
		AllowHeaders:     []string{"Origin", "Content-Type", "Accept", "Authorization"},
		ExposeHeaders:    []string{"Content-Length"},
		AllowCredentials: true,
	}))

	// Auth bounds
	r.POST("/register", api.Register)
	r.POST("/login", api.Login)

	// WebSockets (Token query param is parsed internally in handler)
	r.GET("/ws/points", api.WsPoints)
	r.GET("/ws/tf", api.WsTF)
	r.GET("/ws/video", api.WsVideo)

	// Protected Routes
	protected := r.Group("/")
	protected.Use(api.AuthMiddleware())
	{
		protected.POST("/navigate", api.Navigate)
		protected.GET("/waypoints", api.GetWaypoints)
		protected.POST("/waypoints", api.SaveWaypoint)
		protected.DELETE("/waypoints/:name", api.DeleteWaypoint)

		// Profile Endpoints
		protected.GET("/profile", api.GetProfile)
		protected.POST("/profile", api.UpdateProfile)

		// Mission Planner Endpoints
		protected.POST("/missions", api.CreateMission)
		protected.GET("/missions", api.GetMissions)
		protected.GET("/missions/:id", api.GetMission)
		protected.DELETE("/missions/:id", api.DeleteMission)
		protected.POST("/missions/:id/waypoints", api.AddMissionWaypoint)
		protected.DELETE("/missions/:id/waypoints/:wpId", api.DeleteMissionWaypoint)
		protected.POST("/missions/:id/start", api.StartMission)
		protected.POST("/missions/:id/terminate", api.TerminateMission)
		protected.POST("/missions/:id/mapping/start", api.StartMapping)
		protected.POST("/missions/:id/mapping/stop", api.StopMapping)
	}

	// Server
	log.Println("Go Backend Command Center starting on port 8000...")
	if err := r.Run(":8000"); err != nil {
		log.Fatalf("Server failed: %v", err)
	}
}
