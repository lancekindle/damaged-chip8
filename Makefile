# Define the target filename
TARGET = damaged_chip8

# Define the source assembly file
SOURCE = $(TARGET).asm

# Define the objects
OBJECT = $(TARGET).obj

# Default target
all: assemble link fix clean

assemble:
	@echo "Assembling..."
	rgbasm -o $(OBJECT) $(SOURCE)

link:
	@echo "Linking..."
	rgblink -o chip8.gb $(OBJECT)

fix:
	@echo "Fixing..."
	rgbfix -v -p0 chip8.gb

clean:
	@echo "Removing temporary files..."
	rm -f $(OBJECT)

# Phony targets
.PHONY: all assemble link fix clean

