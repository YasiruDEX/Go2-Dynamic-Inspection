package models

import (
	"gorm.io/gorm"
)

// User represents a user in the database
type User struct {
	gorm.Model
	Username       string `gorm:"uniqueIndex;not null"`
	DisplayName    string `gorm:"type:varchar(100)"`
	HashedPassword string `gorm:"not null"`
	ProfilePic     string `gorm:"type:text"` // Base64 encoded small image
}

// Mission represents a mission plan in the database
type Mission struct {
	gorm.Model
	Name      string            `gorm:"uniqueIndex;not null" json:"name"`
	Status    string            `gorm:"default:created" json:"status"` // created, mapping, ready, running, terminated
	Waypoints []MissionWaypoint `gorm:"foreignKey:MissionID;constraint:OnDelete:CASCADE" json:"waypoints"`
}

// MissionWaypoint represents an ordered waypoint within a mission
type MissionWaypoint struct {
	gorm.Model
	MissionID uint    `gorm:"not null" json:"mission_id"`
	Order     int     `gorm:"not null" json:"order"`
	Name      string  `gorm:"not null" json:"name"`
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Z         float64 `json:"z"`
	Purpose   string  `gorm:"default:none" json:"purpose"` // none, fire_extinguisher, gauge_reading, staircase_start, staircase_end, slope_start, slope_end, human_analyse
}

// MissionCreateRequest is the request payload for creating a mission
type MissionCreateRequest struct {
	Name string `json:"name" binding:"required"`
}

// MissionWaypointCreateRequest is the request payload for adding a waypoint to a mission
type MissionWaypointCreateRequest struct {
	Name    string  `json:"name" binding:"required"`
	X       float64 `json:"x"`
	Y       float64 `json:"y"`
	Z       float64 `json:"z"`
	Purpose string  `json:"purpose"` // none, fire_extinguisher, gauge_reading, staircase_start, staircase_end, slope_start, slope_end, human_analyse
}

// UserProfile represents the public-facing user profile
type UserProfile struct {
	Username    string `json:"username"`
	DisplayName string `json:"display_name"`
	ProfilePic  string `json:"profile_pic"`
}

// UserProfileUpdate is the request to update profile details
type UserProfileUpdate struct {
	DisplayName *string `json:"display_name,omitempty"`
	ProfilePic  *string `json:"profile_pic,omitempty"`
}

// UserCreate is the incoming request payload for registration
type UserCreate struct {
	Username string `json:"username" binding:"required"`
	Password string `json:"password" binding:"required"`
}

// GoalRequest represents the incoming navigation payload
type GoalRequest struct {
	X float64 `json:"x" binding:"required"`
	Y float64 `json:"y" binding:"required"`
	Z float64 `json:"z"`
}

// Waypoint represents a saved location
type Waypoint struct {
	Name string  `json:"name" binding:"required"`
	X    float64 `json:"x" binding:"required"`
	Y    float64 `json:"y" binding:"required"`
	Z    float64 `json:"z" binding:"required"`
}

// SavedWaypoint is the GORM model for saved locations in the database
type SavedWaypoint struct {
	gorm.Model
	Name string  `gorm:"uniqueIndex;not null" json:"name"`
	X    float64 `json:"x"`
	Y    float64 `json:"y"`
	Z    float64 `json:"z"`
}

// MissionResult records inspection outcomes per waypoint per date
type MissionResult struct {
	gorm.Model
	MissionID         uint    `gorm:"not null;index;uniqueIndex:idx_mission_wp_date" json:"mission_id"`
	MissionWaypointID uint    `gorm:"not null;index;uniqueIndex:idx_mission_wp_date" json:"mission_waypoint_id"`
	Date              string  `gorm:"not null;index;uniqueIndex:idx_mission_wp_date" json:"date"` // YYYY-MM-DD
	ImageURL          string  `json:"image_url"`
	Success           string  `gorm:"default:no" json:"success"`     // "yes" or "no"
	Analysis          string  `gorm:"type:text" json:"analysis"`
	Confidence        float64 `gorm:"default:0" json:"confidence"`   // 0.0 - 1.0

	Mission         Mission         `gorm:"foreignKey:MissionID" json:"mission,omitempty"`
	MissionWaypoint MissionWaypoint `gorm:"foreignKey:MissionWaypointID" json:"waypoint,omitempty"`
}

// MissionResultCreateRequest is the request payload
type MissionResultCreateRequest struct {
	MissionWaypointID uint    `json:"mission_waypoint_id" binding:"required"`
	Date              string  `json:"date" binding:"required"`
	ImageURL          string  `json:"image_url"`
	Success           string  `json:"success"`
	Analysis          string  `json:"analysis"`
	Confidence        float64 `json:"confidence"`
}

// ChatRequest is the request payload for the Gemini chat
type ChatRequest struct {
	MissionID uint   `json:"mission_id"`
	Date      string `json:"date"`
	Message   string `json:"message" binding:"required"`
}

// TFAxis defines rotation or translation
type TFAxis struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w,omitempty"` // Only used for rotation
}

// TF represents a transform snapshot
type TF struct {
	Translation TFAxis `json:"translation"`
	Rotation    TFAxis `json:"rotation"`
}

// PathPoint is used for publishing the line strip path
type PathPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// WsTFPayload is the unified payload sent over /ws/tf
type WsTFPayload struct {
	TFs      map[string]TF `json:"tfs"`
	Path     []PathPoint   `json:"path"`
	MqttRate float64       `json:"mqtt_rate"`
}
