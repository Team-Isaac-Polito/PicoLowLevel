# Impostazioni di base
SKETCH_PATH = $(CURDIR)
SKETCH_NAME = $(notdir $(SKETCH_PATH))

# Configurazione board
BOARD_FQBN = rp2040:rp2040:rpipico
OUTPUT_DIR = $(CURDIR)/build/output
BUILD_DIR = $(CURDIR)/build
LIBS_DIR = $(CURDIR)/lib
INCLUDE_DIR = $(CURDIR)/include

# Rilevamento OS
ifeq ($(OS),Windows_NT)
    DETECT_OS = Windows
    BOOTSEL_PATH = E:
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        DETECT_OS = macOS
        BOOTSEL_PATH = /Volumes/RPI-RP2
    else
        DETECT_OS = Linux
        BOOTSEL_PATH = /media/$(USER)/RPI-RP2
    endif
endif

# Comandi PowerShell
PS_CMD = pwsh -Command

# Percorsi librerie
LIBRARY_PATHS = $(wildcard $(LIBS_DIR)/*/src)
LIBRARY_FLAGS = $(addprefix --library ,$(LIBRARY_PATHS))
INCLUDE_PATHS = $(INCLUDE_DIR) $(LIBRARY_PATHS)

# Simboli per output
SUCCESS_MSG = "✅ Compilazione completata!"
ERROR_MSG = "❌ Errore durante la compilazione!"
CLEAN_MSG = "🧹 Pulizia completata!"

# Rilevamento porta COM
PORT = $(shell $(PS_CMD) "(arduino-cli board list --format json | ConvertFrom-Json | Where-Object { $$_.matching_boards.name -match 'Pico' } | Select-Object -First 1).port.address")

# Funzioni di output
define print
	@$(PS_CMD) "Write-Host $1 -ForegroundColor $2"
endef

# Target principali
.PHONY: all compile upload clean help

all: clean compile upload

compile:
	$(call print,$(SUCCESS_MSG),Green)
	@arduino-cli compile --fqbn $(BOARD_FQBN) --build-path $(BUILD_DIR) --output-dir $(OUTPUT_DIR) \
		$(LIBRARY_FLAGS) --build-property "compiler.cpp.extra_flags=$(foreach dir,$(INCLUDE_PATHS),-I$(dir))"

upload:
ifeq ($(PORT),)
	$(call print,"❌ Nessuna board Pico rilevata!",Red)
else
	@$(PS_CMD) "if (Test-Path '$(OUTPUT_DIR)/$(SKETCH_NAME).ino.bin') { \
		arduino-cli upload -p $(PORT) --fqbn $(BOARD_FQBN) --input-dir $(OUTPUT_DIR) } \
		else { Write-Host 'File .bin non trovato!' -ForegroundColor Red }"
endif

upload-bootsel:
	@$(PS_CMD) "if (Test-Path '$(OUTPUT_DIR)/$(SKETCH_NAME).ino.uf2') { \
		try { \
			Copy-Item '$(OUTPUT_DIR)/$(SKETCH_NAME).ino.uf2' '$(BOOTSEL_PATH)' -Force -ErrorAction Stop; \
			Write-Host 'File UF2 copiato in $(BOOTSEL_PATH)' -ForegroundColor Green; \
		} catch { \
			Write-Host 'Errore durante la copia del file UF2!' -ForegroundColor Red; \
		} \
	} else { \
		Write-Host 'File UF2 non trovato!' -ForegroundColor Red; \
	}"


clean:
	@$(PS_CMD) "if (Test-Path $(BUILD_DIR)) { Remove-Item -Recurse -Force $(BUILD_DIR) }"
	$(call print,$(CLEAN_MSG),Green)

monitor:
	@arduino-cli monitor -p $(PORT) -c baudrate=115200

help:
	@$(PS_CMD) "Write-Host 'Comandi disponibili:' -ForegroundColor Cyan; \
		Write-Host '  make all          - Compila e carica il progetto'; \
		Write-Host '  make compile      - Compila il progetto'; \
		Write-Host '  make upload       - Carica il firmware via USB'; \
		Write-Host '  make upload-bootsel - Carica via modalità BOOTSEL'; \
		Write-Host '  make monitor      - Avvia il monitor seriale'; \
		Write-Host '  make clean        - Pulisci i file di build'; \
		Write-Host '  make help         - Mostra questo aiuto'"

auto_com_port:
	@echo "The automatically detected COM port is: $(PORT)"

# List all available COM ports
port:
	@echo "List of COM ports detected by the system:"
	@arduino-cli board list
