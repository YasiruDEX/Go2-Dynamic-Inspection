//go:build ignore

package main

import (
	"fmt"
	"log"

	"golang.org/x/crypto/bcrypt"
	"gorm.io/driver/postgres"
	"gorm.io/gorm"
)

type User struct {
	gorm.Model
	Username       string `gorm:"uniqueIndex;not null"`
	DisplayName    string
	HashedPassword string `gorm:"not null"`
}

func main() {
	dsn := "postgresql://go2_user:FWkKdFsYfFNIDmVxMHF9GRSsz78DjqY6@dpg-d7ogpcapmmbs73b5p7s0-a.singapore-postgres.render.com/go2_db_0hr5"
	db, err := gorm.Open(postgres.Open(dsn), &gorm.Config{})
	if err != nil {
		log.Fatal(err)
	}
	db.AutoMigrate(&User{})

	hash, _ := bcrypt.GenerateFromPassword([]byte("Yasiru1"), bcrypt.DefaultCost)
	user := User{Username: "yasiru@gmail.com", DisplayName: "YASIRU", HashedPassword: string(hash)}
	result := db.Create(&user)
	if result.Error != nil {
		fmt.Println("Error (may already exist):", result.Error)
	} else {
		fmt.Println("User created successfully!")
	}
}
