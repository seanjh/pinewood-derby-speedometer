package main

import (
	"flag"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"time"
)

var portFlag = flag.Int("port", 8080, "Server listen port")

func LoggingMiddleware(h http.HandlerFunc) http.HandlerFunc {
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	return func(w http.ResponseWriter, r *http.Request) {
		logger.Info(
			"Received request",
			slog.String("method", r.Method),
			slog.String("path", r.URL.Path),
			slog.String("host", r.URL.Host),
			slog.String("scheme", r.URL.Scheme))
		h(w, r)
	}
}

func handleHealthCheck(w http.ResponseWriter, _ *http.Request) {
	w.Write([]byte("OK"))
}

func handleSpeedUpdate(w http.ResponseWriter, r *http.Request) {}

func main() {
	flag.Parse()

	http.HandleFunc("GET /health", LoggingMiddleware(handleHealthCheck))
	http.HandleFunc("POST /speed-update", LoggingMiddleware(handleSpeedUpdate))

	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	s := &http.Server{
		Addr:           fmt.Sprintf(":%d", *portFlag),
		ReadTimeout:    5 * time.Second,
		WriteTimeout:   5 * time.Second,
		MaxHeaderBytes: 1 << 20,
	}

	logger.Info("Server starting", "port", *portFlag)
	if err := s.ListenAndServeTLS("server.crt", "server.key"); err != nil {
		logger.Error("Failed to start", "err", err)
		panic(err)
	}
}
