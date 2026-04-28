package database

import (
	"log"
	"mission_planner_backend/models"

	"gorm.io/driver/sqlite"
	"gorm.io/gorm"
)

var DB *gorm.DB

func InitDB(dbUrl string) {
	var err error

	// Temporarily falling back to SQLite because the Render PostgreSQL connection times out locally.
	DB, err = gorm.Open(sqlite.Open("test.db"), &gorm.Config{})
	if err != nil {
		log.Fatalf("Failed to connect to database: %v", err)
	}

	// Migrate the schema
	err = DB.AutoMigrate(&models.User{}, &models.Mission{}, &models.MissionWaypoint{})
	if err != nil {
		log.Fatalf("Failed to migrate database: %v", err)
	}

	log.Println("Database connection established and schema migrated.")
}
