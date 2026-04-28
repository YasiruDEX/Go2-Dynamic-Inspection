package api

import (
	"net/http"

	"mission_planner_backend/auth"
	"mission_planner_backend/database"
	"mission_planner_backend/models"
	"mission_planner_backend/mqttclient"

	"github.com/gin-gonic/gin"
	"gorm.io/gorm"
)

// AuthMiddleware protects routes requiring JWT
func AuthMiddleware() gin.HandlerFunc {
	return func(c *gin.Context) {
		tokenString := c.GetHeader("Authorization")
		if tokenString == "" {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "Authorization header required"})
			return
		}

		if len(tokenString) > 7 && tokenString[:7] == "Bearer " {
			tokenString = tokenString[7:]
		}

		username, err := auth.ValidateToken(tokenString)
		if err != nil {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "Invalid token"})
			return
		}

		c.Set("username", username)
		c.Next()
	}
}

// Register creates a new user
func Register(c *gin.Context) {
	var userReq models.UserCreate
	if err := c.ShouldBindJSON(&userReq); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Check if exists
	var existing models.User
	if err := database.DB.Where("username = ?", userReq.Username).First(&existing).Error; err == nil {
		c.JSON(http.StatusBadRequest, gin.H{"detail": "Username already registered"})
		return
	}

	hashed, err := auth.HashPassword(userReq.Password)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to hash password"})
		return
	}

	user := models.User{
		Username:       userReq.Username,
		HashedPassword: hashed,
	}

	if err := database.DB.Create(&user).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Database error"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"message": "User created successfully"})
}

// Login creates a JWT
func Login(c *gin.Context) {
	// Support OAuth2PasswordRequestForm mimicking FastAPI (x-www-form-urlencoded)
	var form struct {
		Username string `form:"username" binding:"required"`
		Password string `form:"password" binding:"required"`
	}

	if err := c.ShouldBind(&form); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"detail": "Invalid form data"})
		return
	}

	var user models.User
	if err := database.DB.Where("username = ?", form.Username).First(&user).Error; err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"detail": "Incorrect username or password"})
		return
	}

	if !auth.VerifyPassword(form.Password, user.HashedPassword) {
		c.JSON(http.StatusUnauthorized, gin.H{"detail": "Incorrect username or password"})
		return
	}

	token, err := auth.CreateAccessToken(user.Username)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to generate token"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"access_token": token,
		"token_type":   "bearer",
	})
}

// GetProfile returns the current user profile (including base64 profile pic)
func GetProfile(c *gin.Context) {
	username, _ := c.Get("username")

	var user models.User
	if err := database.DB.Where("username = ?", username).First(&user).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "User not found"})
		return
	}

	c.JSON(http.StatusOK, models.UserProfile{
		Username:    user.Username,
		DisplayName: user.DisplayName,
		ProfilePic:  user.ProfilePic,
	})
}

// UpdateProfile updates user profile settings
func UpdateProfile(c *gin.Context) {
	username, _ := c.Get("username")

	var req models.UserProfileUpdate
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	var user models.User
	if err := database.DB.Where("username = ?", username).First(&user).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "User not found"})
		return
	}

	if req.ProfilePic != nil {
		user.ProfilePic = *req.ProfilePic
	}
	if req.DisplayName != nil {
		user.DisplayName = *req.DisplayName
	}

	if err := database.DB.Save(&user).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to update profile"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"message": "Profile updated successfully"})
}

// Navigate sends a goal to MQTT
func Navigate(c *gin.Context) {
	var goal models.GoalRequest
	if err := c.ShouldBindJSON(&goal); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	if err := mqttclient.PublishNavigate(goal); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"status": "Goal sent to MQTT",
		"target": goal,
	})
}

// GetWaypoints reads saved waypoints from the database
func GetWaypoints(c *gin.Context) {
	var waypoints []models.SavedWaypoint
	database.DB.Order("created_at ASC").Find(&waypoints)
	c.JSON(http.StatusOK, waypoints)
}

// SaveWaypoint adds a saved waypoint to the database
func SaveWaypoint(c *gin.Context) {
	var wp models.Waypoint
	if err := c.ShouldBindJSON(&wp); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	saved := models.SavedWaypoint{
		Name: wp.Name,
		X:    wp.X,
		Y:    wp.Y,
		Z:    wp.Z,
	}

	if err := database.DB.Create(&saved).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to save waypoint. Name may already exist."})
		return
	}

	var waypoints []models.SavedWaypoint
	database.DB.Order("created_at ASC").Find(&waypoints)
	c.JSON(http.StatusOK, gin.H{
		"status":    "Saved",
		"waypoints": waypoints,
	})
}

// DeleteWaypoint removes a saved waypoint from the database
func DeleteWaypoint(c *gin.Context) {
	name := c.Param("name")

	result := database.DB.Where("name = ?", name).Delete(&models.SavedWaypoint{})
	if result.RowsAffected == 0 {
		c.JSON(http.StatusNotFound, gin.H{"error": "Waypoint not found"})
		return
	}

	var waypoints []models.SavedWaypoint
	database.DB.Order("created_at ASC").Find(&waypoints)
	c.JSON(http.StatusOK, gin.H{
		"status":    "Deleted",
		"waypoints": waypoints,
	})
}

// --- Mission Planner Handlers ---

// CreateMission creates a new mission
func CreateMission(c *gin.Context) {
	var req models.MissionCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	mission := models.Mission{
		Name:   req.Name,
		Status: "created",
	}

	if err := database.DB.Create(&mission).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to create mission. Name may already exist."})
		return
	}

	c.JSON(http.StatusOK, mission)
}

// GetMissions returns all missions with their waypoints
func GetMissions(c *gin.Context) {
	var missions []models.Mission
	database.DB.Preload("Waypoints", func(db *gorm.DB) *gorm.DB {
		return db.Order("\"order\" ASC")
	}).Order("created_at DESC").Find(&missions)
	c.JSON(http.StatusOK, missions)
}

// GetMission returns a single mission with waypoints
func GetMission(c *gin.Context) {
	id := c.Param("id")
	var mission models.Mission
	if err := database.DB.Preload("Waypoints", func(db *gorm.DB) *gorm.DB {
		return db.Order("\"order\" ASC")
	}).First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}
	c.JSON(http.StatusOK, mission)
}

// DeleteMission deletes a mission and its waypoints
func DeleteMission(c *gin.Context) {
	id := c.Param("id")

	// Delete waypoints first
	database.DB.Where("mission_id = ?", id).Delete(&models.MissionWaypoint{})

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	database.DB.Delete(&mission)
	c.JSON(http.StatusOK, gin.H{"status": "Mission deleted"})
}

// AddMissionWaypoint adds an ordered waypoint to a mission
func AddMissionWaypoint(c *gin.Context) {
	id := c.Param("id")

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	var req models.MissionWaypointCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Determine next order number
	var maxOrder int
	database.DB.Model(&models.MissionWaypoint{}).Where("mission_id = ?", mission.ID).Select("COALESCE(MAX(\"order\"), 0)").Scan(&maxOrder)

	purpose := req.Purpose
	if purpose == "" {
		purpose = "none"
	}

	wp := models.MissionWaypoint{
		MissionID: mission.ID,
		Order:     maxOrder + 1,
		Name:      req.Name,
		X:         req.X,
		Y:         req.Y,
		Z:         req.Z,
		Purpose:   purpose,
	}

	if err := database.DB.Create(&wp).Error; err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to add waypoint"})
		return
	}

	// Return updated mission
	database.DB.Preload("Waypoints", func(db *gorm.DB) *gorm.DB {
		return db.Order("\"order\" ASC")
	}).First(&mission, mission.ID)
	c.JSON(http.StatusOK, mission)
}

// DeleteMissionWaypoint removes a waypoint from a mission
func DeleteMissionWaypoint(c *gin.Context) {
	wpId := c.Param("wpId")

	var wp models.MissionWaypoint
	if err := database.DB.First(&wp, wpId).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Waypoint not found"})
		return
	}

	missionID := wp.MissionID
	database.DB.Delete(&wp)

	// Return updated mission
	var mission models.Mission
	database.DB.Preload("Waypoints", func(db *gorm.DB) *gorm.DB {
		return db.Order("\"order\" ASC")
	}).First(&mission, missionID)
	c.JSON(http.StatusOK, mission)
}

// StartMission sends MQTT start command and updates mission status
func StartMission(c *gin.Context) {
	id := c.Param("id")

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	if err := mqttclient.PublishMissionCommand("start", mission.Name); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to publish MQTT command"})
		return
	}

	mission.Status = "running"
	database.DB.Save(&mission)
	c.JSON(http.StatusOK, gin.H{"status": "Mission started", "mission": mission})
}

// TerminateMission sends MQTT terminate command and updates mission status
func TerminateMission(c *gin.Context) {
	id := c.Param("id")

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	if err := mqttclient.PublishMissionCommand("terminate", mission.Name); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to publish MQTT command"})
		return
	}

	mission.Status = "terminated"
	database.DB.Save(&mission)
	c.JSON(http.StatusOK, gin.H{"status": "Mission terminated", "mission": mission})
}

// StartMapping sends MQTT mapping start command and updates mission status
func StartMapping(c *gin.Context) {
	id := c.Param("id")

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	if err := mqttclient.PublishMappingCommand("start"); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to publish MQTT command"})
		return
	}

	mission.Status = "mapping"
	database.DB.Save(&mission)
	c.JSON(http.StatusOK, gin.H{"status": "Mapping started", "mission": mission})
}

// StopMapping sends MQTT mapping stop command and updates mission status
func StopMapping(c *gin.Context) {
	id := c.Param("id")

	var mission models.Mission
	if err := database.DB.First(&mission, id).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Mission not found"})
		return
	}

	if err := mqttclient.PublishMappingCommand("stop"); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Failed to publish MQTT command"})
		return
	}

	mission.Status = "ready"
	database.DB.Save(&mission)
	c.JSON(http.StatusOK, gin.H{"status": "Mapping stopped", "mission": mission})
}
