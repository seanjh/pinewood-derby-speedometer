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
var certFlag = flag.String("cert", "", "TLS certificate (default: '')")
var certKeyFlag = flag.String("key", "", "TLS certificate key (default: '')")

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

type metersPerSecond = int
type metersPerSecondSquared = int

type speedUpdate struct {
	Timestamp *string          `json:"timestamp"`
	Velocity  *metersPerSecond `json:"velocity"`
}

type errorEntry struct {
	Code    string `json:"code,omitempty"`
	Message string `json:"message,omitempty"`
}

type errorResponse struct {
	Errors []errorEntry `json:"errors,omitempty"`
}

func handleSpeedUpdate(w http.ResponseWriter, r *http.Request) {
	s := speedUpdate{}

	decoder := json.NewDecoder(r.Body)
	if err := decoder.Decode(&s); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	e := []errorEntry{}
	if s.Timestamp == nil {
		e = append(e, errorEntry{
			Code:    "INPUT_VALIDATION",
			Message: "\"timestamp\" is required",
		})
	}
	if s.Velocity == nil {
		e = append(e, errorEntry{
			Code:    "INPUT_VALIDATION",
			Message: "\"velocity\" is required",
		})
	}
	if len(e) > 0 {
		data, err := json.Marshal(errorResponse{
			Errors: e,
		})
		if err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
			return

		}
		http.Error(w, string(data), http.StatusBadRequest)
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

	var err error
	if *certFlag != "" {
		logger.Info("Starting secure server", "port", *portFlag)
		err = s.ListenAndServeTLS(*certFlag, *certKeyFlag)
	} else {
		logger.Info("Starting server", "port", *portFlag)
		err = s.ListenAndServe()
	}
	if err != nil {
		logger.Error("Failed to start", "err", err)
		os.Exit(1)
	}
}
