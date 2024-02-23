package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"net/http/httptest"
	"os"
	"time"
)

var portFlag = flag.Int("port", 8080, "Server listen port")

func LoggingMiddleware(f http.HandlerFunc) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		rec := httptest.NewRecorder()
		f(rec, r)

		resp := rec.Result()
		for k, v := range resp.Header {
			w.Header()[k] = v
		}
		w.WriteHeader(resp.StatusCode)
		n, _ := rec.Body.WriteTo(w)

		slog.Info(
			"Handled request",
			slog.Group("request",
				"method", r.Method,
				"path", r.URL.Path,
			),
			slog.Group("response",
				"status", resp.StatusCode,
				"bytes", n))

	}
}

func handleHealthCheck(w http.ResponseWriter, _ *http.Request) {
	w.WriteHeader(http.StatusOK)
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("OK"))
}

type speedAcceleration struct {
	XMetersPerSecondSquared int `json:"x"`
	YMetersPerSecondSquared int `json:"y"`
	ZMetersPerSecondSquared int `json:"z"`
}

type speedUpdate struct {
	Timestamp               int               `json:"timestamp"`
	VelocityMetersPerSecond int               `json:"veloc"`
	Acceleration            speedAcceleration `json:"accel"`
}

func handleSpeedUpdate(w http.ResponseWriter, r *http.Request) {
	var s speedUpdate

	decoder := json.NewDecoder(r.Body)
	if err := decoder.Decode(&s); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	if s.Timestamp == 0 {
		http.Error(w, "\"timestamp\" is required", http.StatusBadRequest)
		return
	}

	slog.Info("Received speed update", "payload", s)
	w.WriteHeader(http.StatusAccepted)
}

func main() {
	flag.Parse()

	http.HandleFunc("GET /health", LoggingMiddleware(handleHealthCheck))
	http.HandleFunc("POST /speed-update", LoggingMiddleware(handleSpeedUpdate))

	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	slog.SetDefault(logger)
	s := &http.Server{
		Addr:           fmt.Sprintf(":%d", *portFlag),
		ReadTimeout:    5 * time.Second,
		WriteTimeout:   5 * time.Second,
		MaxHeaderBytes: 1 << 20,
	}

	logger.Info("Server starting", "port", *portFlag)
	if err := s.ListenAndServe(); err != nil {
		logger.Error("Failed to start", "err", err)
		os.Exit(1)
	}
}
