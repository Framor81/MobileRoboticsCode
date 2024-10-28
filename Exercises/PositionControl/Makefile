build-dir := build/esp32.esp32.XIAO_ESP32S3
sketch-name := $(wildcard *.ino)

.PHONY: compile
compile: ${build-dir}/${sketch-name}.bin

${build-dir}/${sketch-name}.bin: ${sketch-name} $(wildcard *.h)
	arduino-cli compile . --config-file ../../arduino.yaml

.PHONY: upload
upload: ${build-dir}/${sketch-name}.bin
	arduino-cli upload . --input-dir ${build-dir}

.PHONY: clean
clean:
	rm -rf build
