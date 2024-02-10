ARDUINO_CLI = arduino-cli
BOARD = esp32:esp32:featheresp32

SKETCH_DIR = Speedometer
CONFIG_FILE = $(SKETCH_DIR)/sketch.yaml
SKETCH = $(SKETCH_DIR)/Speedometer.ino
BUILD_DIR = build
TARGET = $(BUILD_DIR)/$(notdir $(SKETCH)).bin
PROFILE = feather

monitor:
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) monitor --port /dev/ttyUSB0

compile: clean
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) compile --output-dir $(BUILD_DIR) $(SKETCH)

upload: compile
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) upload --input-dir $(BUILD_DIR) $(SKETCH)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: compile upload clean
