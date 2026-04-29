package api

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"strings"

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

// --- Mission Result Handlers ---

// GetMissionResults returns results for a mission, optionally filtered by date
func GetMissionResults(c *gin.Context) {
	id := c.Param("id")
	date := c.Query("date")

	query := database.DB.Where("mission_id = ?", id).Preload("MissionWaypoint")
	if date != "" {
		query = query.Where("date = ?", date)
	}

	var results []models.MissionResult
	query.Order("mission_waypoint_id ASC").Find(&results)
	c.JSON(http.StatusOK, results)
}

// GetMissionResultDates returns distinct dates that have results for a mission
func GetMissionResultDates(c *gin.Context) {
	id := c.Param("id")
	var dates []string
	database.DB.Model(&models.MissionResult{}).Where("mission_id = ?", id).Distinct("date").Order("date DESC").Pluck("date", &dates)
	c.JSON(http.StatusOK, dates)
}

// CreateMissionResult creates or upserts a result entry
func CreateMissionResult(c *gin.Context) {
	id := c.Param("id")
	var req models.MissionResultCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	var missionID uint
	fmt.Sscanf(id, "%d", &missionID)

	// Upsert: find existing or create new
	var result models.MissionResult
	err := database.DB.Where("mission_id = ? AND mission_waypoint_id = ? AND date = ?", missionID, req.MissionWaypointID, req.Date).First(&result).Error

	if err == gorm.ErrRecordNotFound {
		result = models.MissionResult{
			MissionID:         missionID,
			MissionWaypointID: req.MissionWaypointID,
			Date:              req.Date,
		}
	}

	result.ImageURL = req.ImageURL
	result.Success = req.Success
	result.Analysis = req.Analysis
	result.Confidence = req.Confidence

	database.DB.Save(&result)
	c.JSON(http.StatusOK, result)
}

// UpdateMissionResult updates an existing result
func UpdateMissionResult(c *gin.Context) {
	resultId := c.Param("resultId")
	var result models.MissionResult
	if err := database.DB.First(&result, resultId).Error; err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "Result not found"})
		return
	}

	var req models.MissionResultCreateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	result.ImageURL = req.ImageURL
	result.Success = req.Success
	result.Analysis = req.Analysis
	result.Confidence = req.Confidence

	database.DB.Save(&result)
	c.JSON(http.StatusOK, result)
}

// DeleteMissionResult deletes a result
func DeleteMissionResult(c *gin.Context) {
	resultId := c.Param("resultId")
	result := database.DB.Delete(&models.MissionResult{}, resultId)
	if result.RowsAffected == 0 {
		c.JSON(http.StatusNotFound, gin.H{"error": "Result not found"})
		return
	}
	c.JSON(http.StatusOK, gin.H{"status": "Deleted"})
}

// ChatWithGemini handles AI chat about mission results
func ChatWithGemini(c *gin.Context) {
	var req models.ChatRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	apiKey := os.Getenv("GEMINI_API_KEY")
	if apiKey == "" || apiKey == "your_gemini_api_key_here" {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Gemini API key not configured"})
		return
	}

	// Build context from mission results
	var contextParts []string

	if req.MissionID > 0 {
		// Specific mission requested
		var mission models.Mission
		database.DB.Preload("Waypoints").First(&mission, req.MissionID)
		contextParts = append(contextParts, fmt.Sprintf("Mission: %s (ID: %d, Status: %s)", mission.Name, mission.ID, mission.Status))

		query := database.DB.Where("mission_id = ?", req.MissionID).Preload("MissionWaypoint")
		if req.Date != "" {
			query = query.Where("date = ?", req.Date)
		}

		var results []models.MissionResult
		query.Find(&results)

		if len(results) > 0 {
			contextParts = append(contextParts, fmt.Sprintf("\nInspection Results for %s:", req.Date))
			for _, r := range results {
				wpName := "Unknown"
				if r.MissionWaypoint.Name != "" {
					wpName = r.MissionWaypoint.Name
				}
				contextParts = append(contextParts,
					fmt.Sprintf(
						"- Waypoint: %s | Purpose: %s | Success: %s | Confidence: %.0f%% | Analysis: %s",
						wpName, r.MissionWaypoint.Purpose, r.Success, r.Confidence*100, r.Analysis,
					))
			}
		} else {
			contextParts = append(contextParts, "No inspection results recorded for this date. The user may not have entered results yet.")
		}

		// Include all available dates for this mission
		var dates []string
		database.DB.Model(&models.MissionResult{}).Where("mission_id = ?", req.MissionID).Distinct("date").Pluck("date", &dates)
		if len(dates) > 0 {
			contextParts = append(contextParts, fmt.Sprintf("\nAll recorded result dates for this mission: %s", strings.Join(dates, ", ")))
		}
	} else {
		// No specific mission — load ALL missions with their most recent results
		var missions []models.Mission
		database.DB.Preload("Waypoints").Order("created_at DESC").Find(&missions)

		if len(missions) == 0 {
			contextParts = append(contextParts, "No missions have been created yet.")
		} else {
			contextParts = append(contextParts, fmt.Sprintf("Total missions in the system: %d", len(missions)))
			for _, m := range missions {
				contextParts = append(contextParts, fmt.Sprintf("\n== Mission: %s (ID: %d, Status: %s, Waypoints: %d) ==",
					m.Name, m.ID, m.Status, len(m.Waypoints)))

				// Get the most recent date with results for this mission
				var latestDate string
				database.DB.Model(&models.MissionResult{}).Where("mission_id = ?", m.ID).
					Distinct("date").Order("date DESC").Limit(1).Pluck("date", &latestDate)

				if latestDate == "" {
					contextParts = append(contextParts, "  No inspection results recorded yet for this mission.")
					continue
				}

				contextParts = append(contextParts, fmt.Sprintf("  Most recent inspection date: %s", latestDate))

				var results []models.MissionResult
				database.DB.Where("mission_id = ? AND date = ?", m.ID, latestDate).Preload("MissionWaypoint").Find(&results)

				for _, r := range results {
					wpName := r.MissionWaypoint.Name
					if wpName == "" {
						wpName = "Unknown"
					}
					contextParts = append(contextParts, fmt.Sprintf(
						"  - Waypoint: %s | Purpose: %s | Result: %s | Confidence: %.0f%% | Analysis: %s",
						wpName, r.MissionWaypoint.Purpose, r.Success, r.Confidence*100, r.Analysis,
					))
				}

				// List all available dates
				var allDates []string
				database.DB.Model(&models.MissionResult{}).Where("mission_id = ?", m.ID).Distinct("date").Pluck("date", &allDates)
				if len(allDates) > 1 {
					contextParts = append(contextParts, fmt.Sprintf("  All recorded dates: %s", strings.Join(allDates, ", ")))
				}
			}
		}
	}

	systemPrompt := `You are an expert mission inspection analyst for a Unitree Go2 quadruped robot that performs daily facility inspections. You have full access to all mission and inspection result data.

Your job:
- Answer questions about inspection outcomes, specific waypoints, dates, and trends
- Summarize results clearly: reference waypoint names, success/failure status, confidence levels, and analysis notes
- Highlight failures or low-confidence readings that need attention
- Compare results across dates when asked
- Be concise, professional, and actionable — no vague responses
- If asked about a specific date or mission not in the context, say so clearly

Here is ALL available mission and inspection data:
` + strings.Join(contextParts, "\n")

	// Call Gemini API
	geminiURL := fmt.Sprintf("https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=%s", apiKey)

	geminiReq := map[string]interface{}{
		"systemInstruction": map[string]interface{}{
			"parts": []map[string]string{
				{"text": systemPrompt},
			},
		},
		"contents": []map[string]interface{}{
			{
				"role": "user",
				"parts": []map[string]string{
					{"text": req.Message},
				},
			},
		},
		"generationConfig": map[string]interface{}{
			"temperature":     0.7,
			"maxOutputTokens": 1024,
		},
	}

	jsonBody, _ := json.Marshal(geminiReq)
	resp, err := http.Post(geminiURL, "application/json", bytes.NewBuffer(jsonBody))
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": fmt.Sprintf("Failed to call Gemini API: %v", err)})
		return
	}
	defer resp.Body.Close()

	body, _ := io.ReadAll(resp.Body)

	var geminiResp map[string]interface{}
	if err := json.Unmarshal(body, &geminiResp); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "Invalid response from Gemini API"})
		return
	}

	// Check for API-level error
	if apiErr, ok := geminiResp["error"].(map[string]interface{}); ok {
		msg, _ := apiErr["message"].(string)
		c.JSON(http.StatusOK, gin.H{"reply": fmt.Sprintf("Gemini API error: %s", msg)})
		return
	}

	// Extract text from candidates
	reply := ""
	if candidates, ok := geminiResp["candidates"].([]interface{}); ok && len(candidates) > 0 {
		candidate := candidates[0].(map[string]interface{})
		// Check finish reason
		if reason, ok := candidate["finishReason"].(string); ok && reason == "SAFETY" {
			reply = "Response was blocked by safety filters. Please rephrase your question."
		} else if content, ok := candidate["content"].(map[string]interface{}); ok {
			if parts, ok := content["parts"].([]interface{}); ok && len(parts) > 0 {
				if text, ok := parts[0].(map[string]interface{})["text"].(string); ok {
					reply = text
				}
			}
		}
	}

	if reply == "" {
		reply = "No response generated. The model may have filtered the content."
	}

	c.JSON(http.StatusOK, gin.H{"reply": reply})
}
