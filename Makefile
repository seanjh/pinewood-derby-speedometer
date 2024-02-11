ARDUINO_CLI = arduino-cli

SKETCH_DIR = Speedometer
SKETCH = $(SKETCH_DIR)/Speedometer.ino
BAUDRATE = 921600
BUILD_DIR = build
TARGET = $(BUILD_DIR)/$(notdir $(SKETCH)).bin

compile: clean
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) compile --output-dir $(BUILD_DIR) $(SKETCH)

upload: compile
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) upload --input-dir $(BUILD_DIR) $(SKETCH)

clean:
	rm -rf $(BUILD_DIR)

monitor:
	@cd $(SKETCH_DIR)
	$(ARDUINO_CLI) monitor --port /dev/ttyUSB0 --config baudrate=$(BAUDRATE)

.PHONY: compile upload clean monitor
