package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"time"
)

var portFlag = flag.Int("port", 8080, "Server listen port")

func LoggingMiddleware(f http.HandlerFunc) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		slog.Info(
			"Received request",
			slog.String("method", r.Method),
			slog.String("path", r.URL.Path),
			slog.String("host", r.URL.Host),
			slog.String("scheme", r.URL.Scheme))
		f(w, r)
	}
}

func handleHealthCheck(w http.ResponseWriter, _ *http.Request) {
	w.WriteHeader(http.StatusOK)
	w.Header().Set("Content-Type", "text/plain")
	w.Write([]byte("OK"))
}

type SpeedUpdate struct {
	Timestamp int `json:"timestamp"`
}

func handleSpeedUpdate(w http.ResponseWriter, r *http.Request) {
	var s SpeedUpdate

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
