include "gbhw.inc"	; wealth of gameboy hardware & addresses info
include "debug.inc"
	bug_verbose	1	; 0 = No output. 1 = msgs & errors. 2 = errors only. 3 = msgs also break

;-------------- INTERRUPT VECTORS ------------------------
; specific memory addresses are called when a hardware interrupt triggers

; Vertical-blank triggers each time the screen finishes drawing. Draw-To-Screen
; routines happen here because Video-RAM is only available during vblank*
SECTION "Vblank", ROM0[$0040]
	jp	vblank_copy_tiles_buffer_to_vram

; LCDC interrupts are LCD-specific interrupts (not including vblank) such as
; interrupting when the gameboy draws a specific horizontal line on-screen
SECTION "LCDC", ROM0[$0048]
	reti

; Timer interrupt is triggered when the timer, rTIMA, ($FF05) overflows.
; rDIV, rTIMA, rTMA, rTAC all control the timer.
SECTION "Timer", ROM0[$0050]
	reti

; Serial interrupt occurs after the gameboy transfers a byte through the
; gameboy link cable.
SECTION "Serial", ROM0[$0058]
	reti

; Joypad interrupt occurs after a button has been pressed. Usually we don't
; enable this, and instead poll the joypad state each vblank
SECTION "Joypad", ROM0[$0060]
	reti
;----------- END INTERRUPT VECTORS -------------------


SECTION "ROM_entry_point", ROM0[$0100]	; ROM is given control from boot here
	nop
	jp	code_begins


;------------- BEGIN ROM HEADER ----------------
; The gameboy reads this info (before handing control over to ROM)
;* macro calls (such as NINTENDO_LOGO) MUST be indented to run
SECTION "rom header", ROM0[$0104]
	NINTENDO_LOGO	; add nintendo logo. Required to run on real hardware
	ROM_HEADER	"  Zchip80 emu  "

include "dma.inc"
include "vars.asm"
include "syntax.inc"
include "memory.asm"
include "lcd.asm"

	var_HighRamByte	TIMER_DELAY
	var_HighRamByte	TIMER_SOUND
	var_HighRamByte	KEY.ACTIVE
	var_HighRamByte	REG.0
	var_HighRamByte	REG.1
	var_HighRamByte	REG.2
	var_HighRamByte	REG.3
	var_HighRamByte	REG.4
	var_HighRamByte	REG.5
	var_HighRamByte	REG.6
	var_HighRamByte	REG.7
	var_HighRamByte	REG.8
	var_HighRamByte	REG.9
	var_HighRamByte	REG.A
	var_HighRamByte	REG.B
	var_HighRamByte	REG.C
	var_HighRamByte	REG.D
	var_HighRamByte	REG.E
	var_HighRamByte	REG.F	; carry Flag register. Set if sprites collide
	var_HighRamByte	REG.I	; index register is 16-bit
	var_HighRamByte	REG.I_LSB
; I grow the stack upward in memory
	var_HighRamByte	REG.SP		; SP is 16-bit
	var_HighRamByte	REG.SP_LSB
	var_HighRamByte	rDE_BIT4	; specific variable for drawing
	var_HighRamByte	jpad_rKeys	; holds last-pressed keys
	var_HighRamByte jpad_rActiveKey	; holds the one chosen active key
	var_HighRamByte	jpad_rHexEncoded
	var_HighRamByte	vram_halfcopy_toggle	; 0/1 for which half of vram to copy
	; this variable (rSP) is not part of chip8's registers
	var_HighRamByte rSP	; holds backup of GameBoy's SP during tile copying operation.
	var_HighRamByte rSP_LSB 
	var_HighRamByte DRAW_Y_COORDINATE	; for detecting vertical wrap when drawing with DXYN
	var_HighRamByte DRAW_X_COORDINATE	; for detecting horizontal wrap when drawing with DXYN
	var_HighRamByte keypad_map	; per each bit, holds a byte 1-F representing 1-16 for a keypress.
	; loaded when loading an new rom
	REPT	7
		var_HighRamByte keypad_map\@
	ENDR

include "joypad.inc"
CHIP8_WIDTH_B = 16
CHIP8_HEIGHT_B = 4

CHIP8_BEGIN = $C000	; a little more than 4KB of memory for chip8 data [$C000 - $D100]
			; including registers, rom, stack, graphics, etc.
CHIP8_FONT = CHIP8_BEGIN	; $C000-$C200 (512bytes) are reserved for fonts/gfx
CHIP8_FONT_END = CHIP8_FONT + $0200
CHIP8_ROM = CHIP8_FONT_END	; actual rom data is loaded at address $C200
CHIP8_PC_BEGIN = CHIP8_ROM 	; program counter begins here, where ROM begins
CHIP8_CALL_STACK = CHIP8_BEGIN + $0EA0  ; call stack / variables: 96 bytes
CHIP8_DISPLAY_TILES = CHIP8_CALL_STACK + $0060	; $CF00 -> $D100 (original before stretch : $CF00 -> $CFFF)
CHIP8_DISPLAY_TILES_END = CHIP8_DISPLAY_TILES + (CHIP8_WIDTH_B * CHIP8_HEIGHT_B * 8) ; $CF00 + $0200
						; holds tiles / display buffer
CHIP8_END = CHIP8_BEGIN + $1100 ; $D100
; set to 1 tile after all chip8-tiles. 64 (8 tiles wide) x 32 (4 tiles tall)
; means that the chip8 needs 32 tiles (0-31). So Tile 32 will be blank
; (used for all other tiles on the screen)
CHIP8_X_OFFSET_B = 4
CHIP8_Y_OFFSET_B = 4
CHIP8_BLANK_TILE = CHIP8_WIDTH_B * CHIP8_HEIGHT_B ; 32

SCREEN_OFFSET_X = (128/2) - (160/2) ; X offset = 64 - 80 = -16
SCREEN_OFFSET_Y = (64/2) - (144/2) ; Y offset = 32 - 72 = -40. Wow that's exactly where we need it
				; because -39 is where we finish copying to VRAM

; screen is 64 x 32. Since the gameboy is 160x144, we can double the size of
; the screen to 128x64. It's still small, but at least it'll look better


; In a nutshell there are two registers: the “sound timer” and “delay timer”.
; Each of these is decremented sixty times per second whenever they are
; non-zero.
; ---
; The delay timer has no special behavior beyond this, but ROMs can read its
; value and use it as a real-time clock.
; ---
; The sound timer cannot be read by ROMs, only written. Whenever its value is
; positive the CHIP-8’s buzzer will sound. That’s the extent of the CHIP-8’s
; sound: one buzzer that’s either on or off.

; safe to include other files here. INCLUDE'd files often immediately add more
; code to the compiled ROM. It's critical that your code does not step over
; the first $0000 - $014E bytes

lda_register: MACRO
; load Chip8 register value into A
; C will hold pointer to required register
	ld	c, LOW(\1)
	ld	a, [c]
	ENDM

; run the following command twice (supports 3 args max currently)
x2: MACRO
	IF _NARG == 1
	\1
	\1
	ENDC
	IF _NARG == 2
	\1, \2
	\1, \2
	ENDC
	IF _NARG == 3
	\1, \2, \3
	\1, \2, \3
	ENDC
	ENDM

; load register pair (I, PC, SP) into HL
; uses A, HL
load_rpair_into_hl: MACRO
	ld	hl, \1
	ldi	a, [hl]	; get MSB of register-pair
	ld	l, [hl]	; get LSB of register-pair
	ld	h, a
	ENDM

; store register pair (I, PC, SP) from HL into ram
; uses A, C, HL
; 9 Cycles. Just as fast as re-using HL as pointer
store_rpair_from_hl: MACRO
	ld	c, LOW(\1)
	ld	a, h
	ld	[$FF00+c], a	; store MSB of register-pair
	inc	c
	ld	a, l
	ld	[$FF00+c], a	; store LSB of register-pair
	ENDM
	

; pushes chip8's PC onto chip8's stack.
; SP += 2
; uses A, C, DE, HL
; trashes chip8's PC from registers (be sure that in-ram value is valid)
push_pc_to_chip8_stack: MACRO
	ldpair	de, hl	; load PC into DE
	load_rpair_into_hl	REG.SP
	inc	hl	; stack mimic's gameboy's: move before write
	ld	[hl], d	; store big-endian: MSB first
	inc	hl
	ld	[hl], e	; store LSB second
	; at this point, BC is overwritten, losing chip8's PC
	; we should push bc, then pop hl to restore PC. But we assume that
	; PC is going to be overwritten since this routine is only called
	; just before a CALL 0NNN function
	store_rpair_from_hl	REG.SP
	ENDM

; "pops" PC from chip8's stack into HL
; SP -= 2
; PC = restored address from stack
; uses A, BC, HL
pop_pc_from_chip8_stack: MACRO
	load_rpair_into_hl	REG.SP
	ld	c, [hl]	; read LSB first
	dec	hl	; SP -= 1
	ld	b, [hl]
	dec	hl	; SP -= 2
	push	bc
	store_rpair_from_hl	REG.SP
	pop	hl	; HL is now restored PC
	ENDM

init_variables:
.zero_registers
	ld	c, LOW(REG.0)
	ld	a, LOW(REG.F)
	sub	c	; A holds # of registers
	inc	a	; increment by 1 to get all registers count
	ld	b, a	; move # of registers to b
	xor	a
.register_zero_loop
	ld	[$FF00+c], a	; set register to zero
	inc	c		; move to next register
	dec	b		; decrement counter
	jr	nz, .register_zero_loop	; repeat until all registers zeroed
.zero_variables
	xor	a
	ld	[jpad_rKeys], a
	ld	[jpad_rHexEncoded], a
	ld	[TIMER_DELAY], a
	ld	[TIMER_SOUND], a
	ld	[vram_halfcopy_toggle], a
	ld	a, -1
	ld	[KEY.ACTIVE], a	; active key needs to be -1 (aka no active key)
	ld	hl, CHIP8_CALL_STACK
	store_rpair_from_hl	REG.SP	; set SP to CHIP8_CALL_STACK. 96 bytes to grow
	ret

copy_rom_to_ram:
	; first copy hex / gfx display data
	ld	hl, hex_gfx_data
	ld	bc, hex_gfx_data_end - hex_gfx_data
	ld	de, CHIP8_FONT
	bug_message	"copying hex gfx %HL% -> %DE%"
	call	mem_Copy
	; eventually I'll make this a macro so I can specify which rom to copy
	; for now we always copy pong rom
	ld	hl, rom_pong
	ld	bc, rom_pong.end - rom_pong
	ld	de, CHIP8_PC_BEGIN
	bug_message	"copying rom %HL% -> %DE% (size %BC%)"
	call	mem_Copy
	bug_message	"... end @ %DE%"
	; now we copy keypad_map
	ld	hl, rom_pong.end	; keypad_map for rom is right after end of rom
	ld	de, keypad_map
	ld	bc, 8	; 8 bytes representing the 8 different key values for D,U,L,R, Start, Select, B,A
	call	mem_Copy
	ret

; setup screen to point to CHIP8 tiles
screen_setup:
	; tile 32 is blank
	; initially set FULL screen to be blank
	call	lcd_Stop
	ld	a, CHIP8_BLANK_TILE	; a points to tile 32 -> a blank tile
	ld	hl, _SCRN0
	ld	b, SCRN_VX_B	; 32
.fill_with_blank
	REPT	SCRN_VY_B	; 32
		ldi	[hl], a
	ENDR
	dec	b
	jr	nz, .fill_with_blank
	; tiles 0-31 should display in a 4x8 tile fashion (4 high, 8 wide)
.point_to_chip8_tiles_onscreen
	xor	a
	ld	hl, _SCRN0
	ld	bc, SCRN_VX_B - CHIP8_WIDTH_B	; aka 32 - 8 because we are chip8 is 8 tiles wide
	REPT	CHIP8_HEIGHT_B	; 4
		REPT	CHIP8_WIDTH_B	; 8
			ldi	[hl], a
			inc	a
		ENDR
		add	hl, bc	; advance to next row. HL now points to next row tile (onscreen) where
				; the next chip8's row begins
	ENDR
.shade_blank_tile
; blank tile fills the rest of the screen. Set it to lightest shade so that chip8 screen is more visible
	ld	hl, _VRAM + CHIP8_BLANK_TILE*16  ; address of blank tile
	ld	a, $FF
	REPT 16	; write 16 bytes (aka one tile's worth). set everything to $FF
		; We'll change %11 to "half-shade" and %01, %10 half-shades to full-dark
		; (basically, differentiate between background tile and CHIP8 tiles)
		ldi	[hl], a
	ENDR
.turn_on_lcd
	call	lcd_On			; turn on lcd
	ldh	a, [rLCDC]
	or	LCDCF_OBJON
	xor	LCDCF_OBJON	; toggle sprites/objects off. Increases H-blank time
	ldh	[rLCDC], a
	call	lcd_ShowBackground	; show background
	call	lcd_ScreenInit	; setup palletes and screen x,y
	ld	a, %01111100	; CHANGE BACKGROUND PALLET TO ... WACKY. %11 = lightly-shaded, whereas %01 or %10 will be full shaded
	ldh	[rBGP], a	;  set background pallet
	ld	a, SCREEN_OFFSET_X
	ldh	[rSCX], a
	ld	a, SCREEN_OFFSET_Y		; set (y,x) to (-40, -16)
	ldh	[rSCY], a			; this'll center the display exactly
	ret
	


code_begins:
	di	; disable interrupts
	ld	SP, $FFFF	; set stack to top of HRAM
	call	init_variables
	call	screen_setup
	ld	a, IEF_VBLANK
	ld	[rIE], a	; enable vblank interrupts
	ld	a, STATF_MODE00	; hblank interrupt
	ld	[rSTAT], a	; enable hblank, when LCDC interrupt is enabled (during vblank we set it)
	; hblank interrupt vector (top of rom) is just RETI. That's purposeful. We actually use the interrupt to resume from a halt
	; during drawing operations.
	call	copy_rom_to_ram
	ei
	ld	hl, CHIP8_PC_BEGIN - 2
	jp	chip8_00E0_disp_clear	; x2 increments HL & clears screen
.loop
	halt	; halts cpu until interrupt triggers
	nop
	jp	.loop

chip8_not_implemented:
	bug_break	"function not implemented"
	jp	chip8.decode_opcode
.pop_pc	; for triggering a bug_break when PC has been pushed to stack
	pop	hl	; pop PC into hl
	jp	chip8_not_implemented


; chip8's vram is a memory address labelled "display refresh" on the wiki
; it exists in the 4k memory reserved for the CHIP8 game, top of the ram
; portion. We copy those tiles to their appropriate VRAM locations (tiles 0-31)
; during vblank
vblank_copy_tiles_buffer_to_vram:
	pushall
	bug_message	"vblank beginning with %CLKS2VBLANK% clocks left"
;	jp	.old_vblank_copy_method

.vblank_copy_routine
	; try copying bytes from GFX to VRAM using SP and popping. we only care about writing every other byte.
	ld	[rSP], SP ; pushes the current Stack Pointer to rSP for safekeeping
	ld	SP, CHIP8_DISPLAY_TILES; (64*32/8)*2 (2 bytes per pixel on 8 gameboy side) = 512 bytes
	ld	HL, _VRAM
	ld	de, 2	; we'll use this to increment HL by 2. We only need to set every other byte in VRAM
	; now we've got SP pointing to start of tiles. And HL points to start of VRAM
	; vblank has 1140 cycles to copy bytes to vram. We use 100*11-cycles aka 1100 of those cycles with the below REPT
	REPT	100  ;(screen is 64x32. But each byte holds 4 pixels, hence /4) aka there's 512 bytes to copy
			; and normally the gameboy uses two bits per pixel. But we don't care! We just write every other
			; tile byte with out pixel data. So we copy 200 bytes here... 312 left during h-blanks
	; sinc we pop off two bytes at a time, we only have to do this loop 256 times! (but vblank isn't long enough for that)
		pop	bc	; pop from chip8 video buffer. increments SP after each read. bc holds two rows of a tile  (3 cycles)
		ld	[hl], c	; (2 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		ld	[hl], b	; (2 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		; total cost = 11 cycles for 2 bytes. Aka 5.5 cycles per byte. Not bad :)
	ENDR
.other_vblank_routine
	ld	hl, rSP
	ldi	a, [hl]
	ld	h, [hl]
	ld	l, a ; HL holds SP
	ld	sp, hl ; restore sp to original location
	call	vblank_handle_timers
.hblank_copy_routine
	di	; disable interrupts. We now start copying half a tile each H-blank from the lower-half of the screen down
	ld	[rSP], SP ; pushes the current Stack Pointer to rSP for safekeeping
	ld	SP, CHIP8_DISPLAY_TILES + (100)*2; (64*32/8)*2 (2 bytes per pixel on 8 gameboy side) = 512 bytes
	ld	HL, _VRAM + (100)*2*2 - 2 ; -2 is for DE initial offset so we can ((add hl, de)) before writing anything
					  ; the extra *2 is because the gameboy's tile data is 16 bytes per tile instead of 8
	ld	de, 2	; we'll use this to increment HL by 2. We only need to set every other byte in VRAM
	ldh	a, [rIE]
	or	IEF_LCDC ; enable LCD interrupts (aka HBLANK)
	ldh	[rIE], a
	; now we've got SP pointing to start of tiles. and HL points to start of VRAM (where we store the tiles drawn onscreen).
	; For 26 h-blanks we fill in 12 bytes each time (with 8 bytes per tile since we only write every-other byte)
	; each h-blank lasts 48.64 microseconds. And that's ~50 cycles. Copying over these 12 bytes takes 61 cycles
	; It may be that this works since the next line starts with LCD performing OAM search (mode 2), and VRAM is
	; still accessible during that time
	ld	a, 0
	REPT	(CHIP8_WIDTH_B*CHIP8_HEIGHT_B*8 - 200)/12  ; finish copying 512 bytes. First 200 copied during vblank, leaving 312 here. We copy 12 bytes per repetition.
			; also since we write every other byte, we actually manage to copy > a full tile each hblank
		ld	[rIF], a ; clear interrupt flags. so H-blank can trigger when it happens  (yeah. it seems that we have to
				 ; clear the flag since the CPU doesn't clear this flag if it doesn't handle it (we don't have 
				 ; interrupts enabled)
		pop	bc	; pop from chip8 video buffer. bc holds two rows of a tile  (3 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		halt	; resumed by hblank interrupt flag. Since we don't enable interrupts, execution stays here
		ld	[hl], c	; (2 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		ld	[hl], b	; (2 cycles)
		REPT 5	; copy 10 more bytes
			pop	bc	; pop from chip8 video buffer. bc holds two rows of a tile  (3 cycles)
			add	hl, de	; increment HL by 2 (2 cycles)
			ld	[hl], c	; (2 cycles)
			add	hl, de	; increment HL by 2 (2 cycles)
			ld	[hl], b	; (2 cycles)
		ENDR
	ENDR
	; finish copying 512 bytes. Since we managed 12 bytes per hblank, we finish the last (312 % 12) bytes here
	; this portion below is just a subset of the above repeated copying cycle
	IF ((CHIP8_WIDTH_B*CHIP8_HEIGHT_B*8 - 200) % 12) > 0	; finish any extra copies not done with above loop.
		ld	[rIF], a ; clear interrupt flags. so H-blank can trigger when it happens  (yeah. it seems that we have to
				 ; clear the flag since the CPU doesn't clear this flag if it doesn't handle it (we don't have 
				 ; interrupts enabled)
		pop	bc	; pop from chip8 video buffer. bc holds two rows of a tile  (3 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		halt	; resumed by hblank interrupt flag. Since we don't enable interrupts, execution stays here
		ld	[hl], c	; (2 cycles)
		add	hl, de	; increment HL by 2 (2 cycles)
		ld	[hl], b	; (2 cycles)
		REPT	((312 % 12) - 2)/2  ; we've already copied 2 bytes of the remainder. Finish copying over the last few bytes
			pop	bc	; pop from chip8 video buffer. bc holds two rows of a tile  (3 cycles)
			add	hl, de	; increment HL by 2 (2 cycles)
			ld	[hl], c	; (2 cycles)
			add	hl, de	; increment HL by 2 (2 cycles)
			ld	[hl], b	; (2 cycles)
		ENDR
	ENDC


.stretch_screen_vertically
; if we repeat the same horizontal line (through rSCY changes) for each line in chip8 we can stretch it vertically
	ld	hl, rLY
	ld	b, 0 - SCREEN_OFFSET_Y	; (screen_offset_y is negative). Taking absolute value would be another alternative
.loop_until_drawing_chip8_onscreen
	ld	a, [hl]
	cp	b
	jr	nz, .loop_until_drawing_chip8_onscreen
	; we get here if on line where chip8 graphics get drawn
	ld	hl, rSCY
	xor	a
.double_each_line
	REPT CHIP8_HEIGHT_B * 8 	; =32. repeat for each row of pixels
		ld	[rIF], a	; clear interrupts
		halt	; will resume on the next hblank interrupt
		ld	[rIF], a	; clear hblank interrupt
		dec	[hl]	; decrement SCY (screen Y position) so graphics line is repeated
		halt	; resume on next hblank (next hblank doesn't stretch)
	ENDR
	ld	[hl], SCREEN_OFFSET_Y	; reset screen position for next screen refresh
	
.return_to_emulation
; restore SP
	ld	hl, rSP
	ldi	a, [hl]
	ld	h, [hl]
	ld	l, a ; HL holds SP
	ld	sp, hl ; restore sp to original locatio
	; now we disable h-blank
	ldh	a, [rIE]
	xor	IEF_LCDC ; disable LCD interrupts (aka HBLANK)
	ldh	[rIE], a
	popall
	reti


chip8:
; all chip8 logic-flow are coded below
; in general, HL contains chip8's PC. Do not expect the REG.PC to be up-to-date
; as such, most routines should jp chip8.decode_opcode after completing their
; instruction
.load_pc
	ld	hl, CHIP8_PC_BEGIN
	jr	.decode_opcode
.pop_pc
	pop	hl	; fxn calls this if they pushed PC before jumping here
.decode_opcode
; assume that [HL] points to next ROM location. AKA HL is the chip8's PC.
; Bytes are stored big-endian so we load MSB first, and compare to determine
; which instruction to run
	bug_message	"NEXT OPCODE AT %HL%"
	ldi	a, [HL]	; HL will point to LSB of opcode
	ifa	>=, HIGH($9000),	jp .decode_9xxx_or_more
.decode_8xxx_or_less
; A = MSB, [HL] points to LSB, just before next opcode
	ifa	<=, HIGH($00EE),	jp .decode_00xx
	ifa	<=, HIGH($0FFF),	jp chip8_0NNN_call_RC1802
	ifa	<=, HIGH($1FFF),	jp chip8_1NNN_jump
	ifa	<=, HIGH($2FFF),	jp chip8_2NNN_call
	ifa	<=, HIGH($3FFF),	jp chip8_3XNN_skip_if_vx_eq_nn
	ifa	<=, HIGH($4FFF),	jp chip8_4XNN_skip_if_vx_not_eq_nn
	ifa	<=, HIGH($5FFF),	jp chip8_5XY0_skip_if_vx_eq_vy
	ifa	<=, HIGH($6FFF),	jp chip8_6XNN_set_vx_to_nn
	ifa	<=, HIGH($7FFF),	jp chip8_7XNN_add_nn_to_vx
	jp	chip8_decode_8xyz
.decode_00xx
; A = MSB, [HL] points to LSB, just before next opcode
	ldd	a, [hl]		; Get LSB. HL will point to MSB of opcode
	; Notice we compare to LOW() byte of 2byte opcode
	ifa	==, LOW($00E0),		jp chip8_00E0_disp_clear
	ifa	==, LOW($00EE),		jp chip8_00EE_return
	; need to restore MSB of opcode into A
	ldi	a, [hl]		; HL += 1 (will point to LSB of opcode)
	jp	chip8_0NNN_call_RC1802 ; opcode is $0NNN
.decode_9xxx_or_more
; A = MSB, [HL] points to LSB, just before next opcode
	ifa	<=, HIGH($9FFF),	jp chip8_9XY0_skip_if_vx_not_eq_vy
	ifa	<=, HIGH($AFFF),	jp chip8_ANNN_set_index_nnn
	ifa	<=, HIGH($BFFF),	jp chip8_BNNN_jump_0NNN_plus_v0
	ifa	<=, HIGH($CFFF),	jp chip8_CXNN_vx_eq_nn_and_random
	ifa	<=, HIGH($DFFF),	jp chip8_DXYN_draw_sprite_xy_n_high
	ifa	<=, HIGH($EFFF),	jp chip8_EXzz_decode
	jp	chip8_FXzz_decode


chip8_0NNN_call_RC1802:
; A contains $0N
; [hl] points to $NN
	ld	b, a
	ldi	a, [hl]	; HL points to next opcode
	ld	c, a
	push	hl
	ld	hl, CHIP8_ROM
	add	hl, bc	; get new address to call
	bug_message	"unimplemented feature: call RC1802 program @ %HL%"
	jp	chip8.pop_pc


; clear screen by setting all relevant tiles to 0. Each "write" takes 2 cycles
chip8_00E0_disp_clear:
; HL points to beginning of its own opcode, so we need to increment twice to
; arrive at next opcode
	bug_message	"00E0_disp_clear"
	inc	hl
	inc	hl	; HL points to next opcode
	push	hl	; save PC

	; we have to write 0 to all the pixels that are part of chip8.
	; if we stick to the resolution of 64x32, it's 8x4 tiles = 32 tiles

	ld	hl, CHIP8_DISPLAY_TILES
	xor	a
	ld	b, 8
.clear_loop
	REPT CHIP8_WIDTH_B * CHIP8_HEIGHT_B 	; =32. repeat once per tile to copy
		ldi	[hl], a	; write 0 to tiles
	ENDR
	dec	b
	jr	nz, .clear_loop
	jp	chip8.pop_pc


chip8_00EE_return:
; return from chip8 subroutine. So we need to restore PC from chip8's stack
	pop_pc_from_chip8_stack		; restores PC to HL
	bug_message	"00EE_return to %HL%"
	jp	chip8.decode_opcode	; decode next opcode from restored PC


chip8_1NNN_jump:
; jump to address MSB, LSB. (aka set PC to address in memory)
; A already holds $1N (MSB), HL points to $NN (LSB of PC)
	and	$0F	; apply mask to get $0N
	bug_message	"1NNN_jump from %HL%"
	ld	c, [hl]	; load $NN in c
	ld	b, a	; BC now contains new PC offset ($0NNN)
	ld	hl, CHIP8_BEGIN
	add	hl, bc	; add address offset to make PC valid
	bug_message	"... by $0NNN (%BC%) to --> %HL%"
	jp	chip8.decode_opcode	; decode next opcode from new PC

chip8_2NNN_call:
; call address 0NNN
; A contains $0N (MSB), HL points to $NN (LSB) of next address
	and	$0F	; apply mask to get $0N
	ld	b, a	; load $0N to B
	ldi	a, [hl]	; HL now points to next opcode after subroutine return
	ld	c, a	; BC now contains $0NNN address to call
	push	bc	; push_pc uses A, C, DE, HL
	push_pc_to_chip8_stack	; preserve PC (HL) on stack
	pop	bc
	ld	hl, CHIP8_BEGIN	; load CHIP8 offset
	add	hl, bc	; add CHIP8 offset to address
	bug_message	"2NNN_call %HL% (offset of %BC%)"
	; jump to new PC since we've pushed previous PC to chip8's stack
	jp	chip8.decode_opcode


; skip next opcode if VX == NN (2nd byte of current opcode)
chip8_3XNN_skip_if_vx_eq_nn:
; A contains $3X. HL points to LSB, which is NN
	and	$0F	; get X reg. offset
	bug_message	"3XNN skip if v%A% == "
	add	LOW(REG.0)	; get LSB of X reg. memory address
	ld	c, a	; [$FF00+c] is now [X]
	ldi	a, [hl]	; HL now points to next opcode
	bug_message	"... %A%"
	ld	b, a	; b stores NN
	ld	a, [$FF00+c]
	; twice increment HL if B (NN) == A (VX). (x2 inc hl skips next opcode)
	ifa	==, b,	x2 inc hl
	ifa	==, b, bug_message	"... SKIPPED"
	jp	chip8.decode_opcode

; skip next opcode if VX != NN (2nd byte of current opcode)
chip8_4XNN_skip_if_vx_not_eq_nn:
; A contains $4X. HL now points to LSB, which is NN
	and	$0F	; get X reg. offset
	bug_message	"4XNN skip if v%A% not =="
	add	LOW(REG.0)	; get LSB of X reg. memory address
	ld	c, a	; [$FF00+c] is now pointer to X register
	ldi	a, [hl]	; HL now points to next opcode
	bug_message	"... %A%"
	ld	b, a	; b stores NN
	ld	a, [$FF00+c]
	; twice increment HL if B (NN) != A (VX). (x2 inc hl skips next opcode)
	ifa	!=, b,	x2 inc hl
	ifa	!=, b, bug_message	"... SKIPPED"
	jp	chip8.decode_opcode

; skip next opcode if VX == VY
chip8_5XY0_skip_if_vx_eq_vy:
; A contains $5X. HL nowpoints to LSB, which is $Y0
	and	$0F	; get X reg. offset
	bug_message	"5XY0 skip if v%A% =="
	add	LOW(REG.0)	; get LSB of X reg. mem address
	ld	c, a	; [X] stored in C
	ld	a, [$FF00+c]	; get VX
	ld	b, a		; store VX in B
	ldi	a, [hl]	; get $Y0 and HL now points to next opcode
	and	$F0	; get Y reg. offset
	bug_message	"... v%A%"
	swap	a	;swap nibbles to get $0Y
	add	LOW(REG.0)	; get LSB of Y reg. memory address
	ld	c, a	; [Y] stored in C
	ld	a, [$FF00+c]	; get VY
	; skip next opcode if VX (B) == VY (A)
	ifa	==, b,	x2 inc hl
	ifa	==, b, bug_message	"... SKIPPED"
	jp	chip8.decode_opcode


; set VX to NN
chip8_6XNN_set_vx_to_nn:
; HL points to NN, A contains $6X
	and	$0F		; get register X offset
	bug_message	"6XNN set v%A% to "
	add	LOW(REG.0)	; get LSB of reg. X memory address
	ld	c, a		; stow [X] in c
	ldi	a, [hl]	; A = NN. [HL] now points to next opcode
	bug_message	"... %A%"
	ld	[$FF00+c], a
	jp	chip8.decode_opcode

; set VX += NN (carry flag is not changed)
chip8_7XNN_add_nn_to_vx:
; HL points to NN, A contains $7X
	and	$0F		; get reg. X offset
	bug_message	"7XNN add ... to v%A%"
	add	LOW(REG.0)	; get LSB of reg. X memory address
	ld	c, a		; stow [X]
	ld	a, [$FF00+c]	; get VX
	bug_message	"... vX prev == %A%"
	add	[hl]	; add NN to VX
	bug_message	"... vX now == %A%"
	inc	hl	; HL now points to next opcode
	ld	[$FF00+c], a	; store VX + NN in [X]
	jp	chip8.decode_opcode

chip8_decode_8xyz:
; HL points to LSB of opcode
; A holds 8X
	bug_message	"8xyz"
	and	$0F	; mask to get X register
	add	LOW(REG.0)	; add REG.0 offset
	ld	b, a	; store [X] in B
	ldi	a, [hl]	; get $Y# and [HL] now points to next opcode
	ld	e, a	; store backup of $Y#
	and	$F0	; mask to get Y register as $Y0
	swap	a	; swap nibbles to get $0Y
	add	LOW(REG.0)	; add REG.0 offset
	ld	c, a	; store [Y] in c
	; BC now holds [X],[Y]  (assuming X == LSB of $FFXX)
	push	hl	; store PC pointing to next opcode
	ld	hl, .vector_table
	ld	a, e	; restore $Y#
	and	$0F	; apply mask to get $0# (subroutine #)
	; jump table contains 2 bytes per entry (a 16-bit subroutine address)
	; so we need to multiply subroutine # (reg. A) by 2 to get
	; vector_table offset
	add	a	; A = 2x
	add	l		; <\
	ld	l, a		;   | HL += A
	if_flag	c, inc h	; </
	; HL now points to jump address in jump table
	ldi	a, [hl]		; load LSB of jump address
	ld	h, [hl]		; load MSB of jump address
	ld	l, a		; move LSB to L
	; HL now points to correct 8xy# subroutine
	ld	a, [$FF00+c]	; load VY
	ld	c, b		; load [X] in C
	ld	b, a		; store VY in B
	ld	a, [$FF00+c]	; load VX in A
	; now: A holds VX, B holds VY, C holds [X]
	bug_message	"8xyz => 8%A%%B%z"
	jp	hl	; jump to decoded 8XY# routine
.vector_table
; vector table contains 16-bit memory address of subroutines 0-F. Simply add
; subroutine # twice to vector_table address and load 16bit address from there
; into HL, then JP HL to begin the desired subroutine
; (this is NOT a jump table. A jump table contains the full "jp .addr" opcode)
	DW	.chip8_8XY0_vx_eq_vy
	DW	.chip8_8XY1_vx_eq_vx_or_vy
	DW	.chip8_8XY2_vx_eq_vx_and_vy
	DW	.chip8_8XY3_vx_eq_vx_xor_vy
	DW	.chip8_8XY4_add_vy_to_vx
	DW	.chip8_8XY5_sub_vy_from_vx
	DW	.chip8_8XY6_shift_vx_right
	DW	.chip8_8XY7_vx_eq_vy_minus_vx
	DW	.chip8_8XYZ_not_implemented	; .88XY8
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XY9
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XYA
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XYB
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XYC
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XYD
	DW	.chip8_8XYE_shift_vx_left
	DW	.chip8_8XYZ_not_implemented	; .chip8_8XYF
; remember. At this point
; A => VX
; BC => VY, [X]
; PC has been pushed to gameboy stack
.chip8_8XYZ_not_implemented
; we get here if it broke. gracefully print message and move to next opcode
	ld	bc, .vector_table	; overwrite all values
	negate	bc
	add	hl, bc	; calculate vector_table offset (aka routine # ?)
	bug_break	"8XY? routine called nonimplemented version: %HL%"
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY0_vx_eq_vy
	ld	a, b		; load VY into A
	ld	[$FF00+c], a	; [X] = VY
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY1_vx_eq_vx_or_vy
	or	b
	ld	[$FF00+c], a	; [X] = VX | VY
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY2_vx_eq_vx_and_vy
	and	b
	ld	[$FF00+c], a	; [X] = VX & VY
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY3_vx_eq_vx_xor_vy
	xor	b
	ld	[$FF00+c], a	; [X] = VX ^ VY
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY4_add_vy_to_vx	; VF set if overflow
	add	b
	ld	[$FF00+c], a	; [X] = VX + VY
	ld	hl, REG.F		; <\
	ld	[hl], 0			;   | set Carry if overflow
	if_flag	c,	inc [hl]	; </
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY5_sub_vy_from_vx
	sub	b
	ld	[$FF00+c], a	; [X] = VX - VY
	ld	hl, REG.F		; <\  clear Carry if overflow / borrow
	ld	[hl], 1			;   | (subtraction is opposite to
	if_flag	c,	dec [hl]	; </  addition concerning carry flag)
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY6_shift_vx_right
	srl	a	; rotate VX right
	ld	[$FF00+c], a	; [X] = VX >> 1
	ld	hl, REG.F		; <\
	ld	[hl], 0			;   | VF = bit-value shifted out of VX
	if_flag	c,	inc [hl]	; </
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XY7_vx_eq_vy_minus_vx
; beware the trap of using negate macro. Carry-flags are reversed, then
	ld	e, a	; store VX
	ld	a, b	; A = VY
	sub	e	; A = VY - VX
	ld	[$FF00+c], a	; [X] = VX - VY
	ld	hl, REG.F		; <\  clear Carry if overflow / borrow
	ld	[hl], 1			;   | (subtraction is opposite to
	if_flag	c,	dec [hl]	; </  addition concerning carry flag)
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL
.chip8_8XYE_shift_vx_left
	sla	a	; rotate VX left
	ld	[$FF00+c], a	; [X] = VX << 1
	ld	hl, REG.F		; <\
	ld	[hl], 0			;   | VF = bit-value shifted out of VX
	if_flag	c,	inc [hl]	; </
	jp	chip8.pop_pc	; return to next opcode, but first pop PC to HL


chip8_9XY0_skip_if_vx_not_eq_vy:
; A contains $9X, HL points to $Y0
	bug_message	"9xy0 skip if vx not eq vy"
	and	$0F	; get X register offset from $9X
	add	LOW(REG.0)	; get LSB of [X]
	ld	c, a	; store [X]
	ld	a, [$FF00+c]	; get VX
	ld	b, a		; store VX
	ldi	a, [hl]	; A = $Y0, HL points to next opcode
	and	$F0
	swap	a	; swap nibbles to get $0Y
	add	LOW(REG.0)	; get LSB of [Y]
	ld	c, a
	ld	a, [$FF00+c]	; get VY
	; A = VY, B = VX
	; if VX != VY, skip next opcode (aka "inc hl" twice)
	ifa	!=, b,	x2 inc hl
	ifa	!=, b, bug_message	"... SKIPPED"
	jp	chip8.decode_opcode

; set index to $0NNN + chip8_rom offset (actually chip8_data offset)
chip8_ANNN_set_index_nnn:
; A contains $AN, HL points to $NN
	and	$0F	; get $0N
	ld	b, a
	ldi	a, [hl]	; get $NN, HL now points to next opcode
	ld	c, a	; BC now holds $0NNN
	push	hl
	ld	hl, CHIP8_BEGIN
	add	hl, bc	; add CHIP8 offset. Otherwise REG.I will not be valid
	bug_message	"ANNN set index += %BC% aka %HL%"
	store_rpair_from_hl	REG.I
	pop	hl	; restore PC
	jp	chip8.decode_opcode


chip8_BNNN_jump_0NNN_plus_v0:
; A contains $BN, HL points to NN
	and	$0F	; get $0N
	ld	l, [hl]	; load $NN into L. We're jumping. So overwrite HL away!
	ld	h, a
	; HL now contains $0NNN
	ld	c, LOW(REG.0)
	bug_message	"BNNN jump %HL% plus REG.%C%"
	ld	a, [$FF00+c]	; load v0
	ld	c, a
	xor	a	; zero A
	ld	b, a
	; BC now contains $00VV (VV = V0 register value)
	add	hl, bc	; add V0 to $0NNN
	ld	de, CHIP8_BEGIN
	add	hl, de	; add chip8 rom offset
	bug_message	"... HL + %C% + CHIP8-rom @ %DE%--> %HL%"
	; HL now contains new PC from jump
	jp	chip8.decode_opcode

; AND $NN with a random number, store in VX
chip8_CXNN_vx_eq_nn_and_random:
; A contains $CX, HL points to NN
	bug_message	"CXNN vx eq nn + random"
	and	$0F	; get X register offset
	add	LOW(REG.0)	; add to REG.0
	ld	c, a	; c holds [X]
	ldi	a, [hl]	; A holds $NN, HL points to next opcode
	ld	b, a	; store $NN in B
	; get state of divisor register on gameboy as random input
	; rDIV will be our random input. Always runs @ 16,384Hz
	ldh	a, [rDIV]
	and	b	; AND $NN with random rDIV input
	ld	[$FF00+c], a	; store random-ish # in VX
	jp	chip8.decode_opcode


; set F to 1. Overwrites A
SET_REG_F: MACRO
	ld	a, 1
	ld	[REG.F], a
	ENDM

chip8_DXYN_draw_sprite_xy_n_high:
; draw sprite @ Vx,Vy with width of 8 pixels, height of N pixels (max 16)
; X, Y is wrapped around screen, such that X = X % 64 and Y = Y % 32
; each row of pixels is read as bit-coded pixels starting at memory location
; REG.I. REG.I doesn't change after this operation. VF is set to 1 if any
; pixels are flipped from 1 to 0 as a result of this operation (pixels are
; XOR'd onto screen) which indicates sprite collision. REG.I points to sprite?
;==
; this is a complicated function because the graphical memory representation
; on the gameboy is of 4x8 tiles. Thus the gameboy screen really looks like:
;  _______________    where each square is really 8x8 pixels. Each row is 8 pixels
; |_|_|_|_|_|_|_|_|   And since we draw 8 pixels per row, there's a large
; |_|_|_|_|_|_|_|_|   possibility that we draw on at least two tiles on any given
; |_|_|_|_|_|_|_|_|   drawing row. (you can't expect drawing to often line up
; |_|_|_|_|_|_|_|_|   with a tile)
;
; Another thing to note: We may traverse to a tile BELOW our current tile if we
; finish drawing on the bottom of a tile. Which requires adding a full row to our
; memory address. Aka 8 bytes (but then subtract adjustments for bottom vs top of tile)
;==
; A contains $DX, [HL] points to $YN
	bug_message	"DXYN drawing begins"
	and	$0F	; mask to get $0X offset
	bug_message	"... X -> REG.%A%"
	add	LOW(REG.0)	;add REG.0 to offset
	ld	c, a	; [X] in [C]
	ld	a, [$FF00+c]	; get VX
	; wrap X around screen. aka X % 64
	; this is actually a binary operation masking off any bits >=64
	; since 64 can be represented by 1 bit, and any bits greater than that (aka bit 7=128)
	; is also a multiple of 64
	and	63	; aka AND %00111111. This is exactly the same as %64
	ld	[DRAW_X_COORDINATE], a	; store current X. For on-screen wrapping
	bug_message	"... vX = %A%"
	ld	b, a	; store VX
	ld	a, [hl]	; get $YN, but do NOT increment HL.
	; (we will re-read $YN to get $0N later)
	and	$F0	; mask to get $Y0
	swap	a	; swap to get $0Y
	bug_message	"... Y -> %A%"
	add	LOW(REG.0)	; add REG.0 to offset to get full address
	ld	c, a
	ld	a, [$FF00+c]	; get VY
	; wrap Y around scren aka Y % 32
	; this is also a binary operation masking off any bits >= 32
	and	31	; aka AND %00011111
	ld	[DRAW_Y_COORDINATE], a	; store current Y. For on-screen wrapping
	bug_message	"... vY = %A%"
	ld	c, a	; store VY in C
	; BC contains $VX,VY   where VX = value of [X], and VY = value of [Y]
	; [HL] points to $YN (so we can get height)
	ldi	a, [hl]	; load $YN. HL now points to next opcode
	push	hl	; store next opcode pointer
	and	$0F	; get $0N
	bug_message	"... $0N = %A%"
	push	af	; preserve $0N
	; reset VF (required at start of drawing operation)
	xor	a
	ldh	[REG.F], a
.get_Y_tile_offset
	; DE will point to tile 0. Each tile is 8x4 pixels, 8 bytes per tile
	; CRITICAL. YES, 8 Bytes per tile. CHIP8 is B/W, so we store the tiles
	; in ram as 8 bytes each, and double-copy each byte into VRAM.
	; So we need to jump down 1 tile Y/8 times (jumping down a tile is
	; jumping a full row of tiles. Since we treat each row as 16 tiles, 16
	; byte per row, we inc DE by 8x16, or 128 Bytes)
	; So it's really VY / 8, then x128
	; or... VY >> 3, then VY << 7. OR (VY & %11111000) << 4
	ld	a, c	; get VY
	and	%11111000	; aka (A >> 3) << 3
	; now we multiply A by 8 (aka << 3)
	ld	l, a
	ld	h, 0
	; HL => A (HL stores A)
	add	hl, hl	; HL = 2A
	add	hl, hl	; HL = 4A
	add	hl, hl	; HL = 8A
	add	hl, hl	; HL = 16A (aka HL = A << 4)
	; tadah. now [HL] points to correct tile row.
	; (but not correct pixel row within tile)
	ld	a, c	; restore VY
	and	%00000111	; get first 3 bits only of VY
	; A now holds 0-7 aka fine adjustment within the tile of 8 bytes
	add	l
	if_flag	c, inc	h	; add first 3 bits of VY to HL
	ld	l, a	; now [HL] points to correct tile row and correct
			; pixel row of tile. BUT it does not yet point to
			; correct tile COLUMN
	push	hl	; store Y-corrected version of tile-adjusted address
	bug_message	"... y tile offset=%HL%"
.get_X_tile_offset
	; now we need to get tile offset according to X coordinates
	; which since each tile is 4 pixels wide
	; but then since each tile is 8 bytes, we divide by 4 to get tiles to skip
	; then multiply by 8 to account for each tile's # of bytes
	; so we basically need to A/4, then A*8 (aka a >> 2, then a << 3)
	; which is really same as "AND %11111100" << 1
	ld	a, b	; get VX
	and	%11111100
	ld	l, a
	ld	h, 0
	add	hl, hl	; HL = A << 1
	; VX ==> [HL] now contains X offset
	bug_message	"... x tile offset=%HL%"
	pop	de	; pop Y tile's offset
	add	hl, de	; add Y's offset to X's offset
	ld	de, CHIP8_DISPLAY_TILES	; this is where this routine draws
					; during vblank this is copied to VRAM
	add	hl, de	; add (tile) offset. Now HL points to correct
	bug_message	"... correct tile address=%HL%"
	; tile (via row, column), and the correct pixel row within the correct
	; tile
	; but the correct pixel within the correct row? that will be pointed at
	; by a pixel mask
	ldpair	de, hl	; load tile pointer into DE
	load_rpair_into_hl	REG.I	; overwrites A, HL
	bug_message	"... sprite to begin drawing from %HL% to %DE%"
.get_bitmask
	; now GET that PIXEL BITMASK!
	; VX is not fully accounted for. Within the correct tile, and the
	; correct row of pixels, we still need to start at a pixel.
	; (currently we'd start at pixel 0 within the tile)
	; this is where we compute a bitmask to represent the starting pixel
	; X coordinate. The bitmask will move right as we sample bits to the
	; right
	ld	a, b		; load VX
	and	%00000011
	; basically, we need to turn the 2 bit value into a single
	; bit toggled on that represents bitmask 0-3
	; we are drawing to two bits per pixel, since we are stretching the
	; graphics horizontally
	; if A == 0, that means we start at beginning of tile... %11000000
	; so the bitmask is reversed compared to the value

	; CASE mimics IFA but jumps to .endcase after a match occurs
	case	==, 0, .endcase, ld	b, %11000000
	case	==, 1, .endcase, ld	b, %00110000
	case	==, 2, .endcase, ld	b, %00001100
	case	==, 3, .endcase, ld	b, %00000011

.endcase
	pop	af	; restore $0N in A
	; ASSUME here that B contains bitmask
	; ASSUME that HL contains REG.I (so it's pointing to sprite to draw)
	; ASSUME that [DE] points to CHIP8_DISPLAY / BUFFER tile / pixel row
	; ASSUME that A contains N (height of sprite to be drawn)
	inc	a
	dec	a
	jp	z, .done_drawing_sprite
	push	af	; store # of rows to draw
	ld	a, e
	and	%00001000	; get bit 4 of DE
	ld	[rDE_BIT4], a	; set last known bit 4 of DE
	; bit4 only changes if we accidentally roll from a tile to the tile
	; to the right of the current tile. We check this to determine if we
	; need to jump from bottom of a tile to top of the tile below it
	jp	.draw_sprite_row_loop
draw_pixel: MACRO
.pixel_\1_\@
	ld	a, [DRAW_X_COORDINATE]
	inc	a	; increment our known X coordinate (for later bound-check
	ld	[DRAW_X_COORDINATE], a				 ; & wrap-around)
	bit	\1, [hl] ; test bit x. If it's 0 we don't have to do anything
			 ; if it's 1, we xor current mask with [de]
	jp	z, .rotate_mask_\@
	ld	a, [de]	; load vram pixels (4 pixels at a time), 1 bit of shading
			; two bits represesent one chip8 pixel
	ld	c, a	; store copy of vram pixels
	xor	b	; xor bitmask to toggle on / off pixel
	; If bit is zeroed from the xor operation, need to set REG.F
	ld	[de], a	; set VRAM pixel
	ifa	<, c, SET_REG_F	; A < C if pixel was set to zero / toggled off
.rotate_mask_\@
	rrc	b
	rrc	b	; rotate mask by two bits. If it rotates back to
			; bits 7-6 we need to jump to next tile
	call	c, .draw_jumps_to_next_tile
	ENDM

.draw_jumps_to_next_tile
	; jump to next tile's same row. Since each tile is 8 rows of 1 byte
	; each, we add 8 to our tile pointer
	ld	a, e
	add	8
	ld	e, a
	if_flag	c, inc	d
	; [DE] now points to next tile's row
.wrap_horizontally
	; check if X >= 64, in which case we subtract a full width of tiles
	; from our address to end up from out-of-range-to-right to beginning-of-left
	ld	a, [DRAW_X_COORDINATE]
	ifa	<, 64,	ret	; return if X is in-bounds
	; we get here if X == 64 (technically >= 64. But due to when we check it's always == 64)
	ld	a, 0
	ld	[DRAW_X_COORDINATE], a	; reset X coordinate
	; so [DE] needs to be adjusted by a full width of tiles.
	push	hl
	ld	hl, -(CHIP8_WIDTH_B * 8)
	add	hl, de
	ldpair	de, hl	; [DE] has been wrapped from beyond-rightmost-tile to start of leftmost-tile
			; on the chip8 screen
	pop	hl
	ret

.draw_sprite_row_loop
	draw_pixel	7
	draw_pixel	6
	draw_pixel	5
	draw_pixel	4
	draw_pixel	3
	draw_pixel	2
	draw_pixel	1
	draw_pixel	0
.check_done_drawing
	; check if done drawing
	pop	af
	dec	a
	jp	z, .done_drawing_sprite
	push	af	; if not done, re-push number of rows left to draw

.adjust_DE_back_to_next_row
	; since .draw_jumps_to_next_tile happens TWICE (at some point)
	; after all 8 pixels have been drawn, we ALWAYS have to rewind
	; backwards two tiles, then jump down one row on same tile
	; so we subtract 16 (rewind 2 tiles) from DE, then add 1 (jump 1 row
	; down) So DE = DE - 16 + 1; DE -= 15
	ld	a, e
	sub	15
	ld	e, a
	; the above subtract 15 only causes an underflow if E < 15
	if_flag	c,	dec d
	;[DE] now points to next row's pixels (of tile)
	; we need [HL] (aka REG.I) to point to next row of sprite's pixels
	inc	hl
.check_horizontal_wrap
	; if we wrapped around screen horizontally (while drawing ACROSS). Then we need to unwrap
	; for the next row (as X will be reset). Since X += 8 always, we can check this and undo
	ld	a, [DRAW_X_COORDINATE]
	ifa	<=, 7,	jr .undo_horizontal_wrap
.did_not_horizontal_wrap
	; X is incremented for each pixel in row, so we remove 8 to re-adjust for new row
	sub	8
	ld	[DRAW_X_COORDINATE], a
	jr	.check_DE_migration
.undo_horizontal_wrap
	; we get here if A <= 7 aka A was wrapped around after a pixel or more had already drawn
	; we need to reset A, and reset [DE] from our address wrap-around
	sub	8
	add	64
	ld	[DRAW_X_COORDINATE], a	; reset X coordinate to original value
	push	hl
	ld	hl, CHIP8_WIDTH_B * 8	; advance forward to unwrapped X address
	add	hl, de
	ldpair	de, hl	; [DE] is rewound from wrap-around
	pop	hl
.check_DE_migration
	; OK but at this point, if [DE] jumps from bottom of tile to top of
	; next tile, that's BAD. We need to detect that. We wanted, instead
	; to have jumpted from bottom of tile to the top of the tile below
	; [DE]'s starting tile. So we'd add 16-1 tiles (15x8=120) to [DE] IFF
	; it accidentally jumped to top of next tile. That only happens if
	; bit 4 of E toggles/changes.
	; Bit 4 == 8. Aka when bit 4 toggles, we've advanced >= 8 bytes.
	; and since each tile is 8 bytes, we know we're on a new tile
	ld	a, e
	and	%00001000	; get bit 4 of E (and of DE in general)
	ld	c, a	; store bit 4 of DE
	ld	a, [rDE_BIT4]	; get last known bit 4 of DE
	xor	c	; xor with DE's original bit4. If it's nonzero, then 
			; bit 4 of DE was changed!
	jp	z, .draw_sprite_row_loop	; jump if we didn't move to another tile
.adjust_DE_to_top_of_tile_below
	; if we get to here, we need to add 7 tiles' worth of bytes to DE
	; to make up for the fact that DE just moved from bottom of one tile
	; to top of tile on the right. (We want to move it so that it ends up
	; at top of tile below it, instead: so 16 tiles - 1 (because we're
	; already 1 tile ahead)). 16-1 tiles x 8 bytes per tile = 15x8 = 120
	ld	a, e	; load LSB of tile pointer
	add	(CHIP8_WIDTH_B - 1) * 8 ; =120. 8 bytes per tile
	ld	e, a
	if_flag	c,	inc d	; MSB + 1 if necessary
	; [DE] now points to top of tile below starting tile (if organized in a square grid of 16x16 tiles)
	ld	a, e
	and	%00001000	; get bit 4 of E (since it may have changed)
	ld	[rDE_BIT4], a	; set new bit 4 of DE
.wrap_around_screen_vertically
	; check if new row is >= 32, in which case we need to adjust which tile we are drawing on AGAIN
	ld	a, [DRAW_Y_COORDINATE]
	inc	a
	ld	[DRAW_Y_COORDINATE], a
	ifa	<, 32,	jp .draw_sprite_row_loop	; no wrap around needed. Carry on drawing
	; we get here if Y >= 32, in which case we need to wrap Y value and DE appropriately
	ld	a, 0
	ld	[DRAW_Y_COORDINATE], a	; wrap around always back to 0
	push	hl	; store sprite address
	ld	hl, -(CHIP8_WIDTH_B * CHIP8_HEIGHT_B * 8)
	add	hl, de
	ldpair	de, hl	; DE = DE - CHIP8_SCREEN. Aka bump address up full screen's-worth.
			; should properly adjust address from out-of-range bottom to top of screen
	pop	hl	; restore sprite address
	jp	.draw_sprite_row_loop
.done_drawing_sprite
	ldh	a, [REG.F]
	; it appears that drawing dictates the speed of the game. So not
	; pausing "overspeeds" the game. But pausing after every draw causes
	; sprites to flicker. On CRT / bleeding screens, this is OK, since
	; the sprites will be more gray instead of pure black / white.
	; So my compromise (for now) is to pause only when F flag is unset
	; Most sprite movements are two draws: erase, then update and draw
	; so if I only pause on the update and draw time, it's less likely
	; to catch the flickering of sprites.
	; BUT ITS ALSO PROBABLY GOING TO INTRODUCE SPORADIC SPEEDUPS IN GAME.
	;halt
	ifa	==, 0, halt ; WARNING CRITICAL: THIS VASTLY SLOWS DOWN THE GAME
	jp	chip8.pop_pc


; keycode instructions: check how VX relates to pressed key
chip8_EXzz_decode:
	bug_message	"EXzz keycodes"
; A holds $EX, HL points to $ZZ (either $9E or $A1)
	and	$0F	; get [X] offset
	add	LOW(REG.0)	; get register [X] address
	ld	c, a	; c holds [X]
	ldi	a, [hl]	; A=$9E or $A1, HL points to next opcode
	ifa	==, $9E, jp .chip8_EX9E_skip_if_vx_eq_key
	ifa	!=, $A1, bug_break "EX%A% not a valid opcode"
; skip next opcode if key stored in REG.X not equal to key pressed (if any)
.chip8_EXA1_skip_if_vx_not_eq_key
	; A holds $A1, C holds [X]
	push	bc
	push	hl
	call	get_key_press	; overwrites A,B,C, HL
	pop	hl
	pop	bc
	ld	b, a	; move keypress to b
	ld	a, [$FF00+c]	; load vx
	bug_message	"EXA1 skip if vx (%A%) not eq to key (%B%)"
	ifa	!=, b, x2 inc hl	; skip next opcode
	ifa	!=, b, bug_message "... next opcode skipped"
	jp	chip8.decode_opcode
; skip next opcode if key pressed (0-F) equals VX
.chip8_EX9E_skip_if_vx_eq_key
	push	bc
	push	hl
	call	get_key_press	; overwrites A,B,C?
	pop	hl
	pop	bc
	ld	b, a	; move keypress to b
	ld	a, [$FF00+c]	; load vx
	bug_message	"EX9E skip if vx (%A%) eq to key (%B%)"
	ifa	==, b, x2 inc hl	; skip next opcode
	ifa	==, b, bug_message "... next opcode skipped"
	jp	chip8.decode_opcode


; return keypress. 255 (-1) if none. 0-F if a key pressed
get_key_press:
	; arrow keys + A + B are keys to press
	; start/select modify the keymappings such that
	; the full 16 keys 0-F are represented when
	; start or select or no modifier is held down
	jpad_GetKeys	; a huge macro
	ret



chip8_FXzz_decode:
; A holds $FX, HL points to $ZZ (can be several variants)
	and	$0F	; get X register offset
	bug_message	"FXzz (F%A%zz)"
	add	LOW(REG.0)	; get [x] register
	ld	c, a	; c holds [x]
	ldi	a, [hl]	; A = $??, HL points to next opcode
	ifa	==, $07,	jp .chip8_FX07_vx_eq_delay_timer
	ifa	==, $0A,	jp .chip8_FX0A_vx_eq_key_pressed
	ifa	==, $15,	jp .chip8_FX15_delay_timer_eq_vx
	ifa	==, $18,	jp .chip8_FX18_sound_timer_eq_vx
	ifa	==, $1E,	jp .chip8_FX1E_add_vx_to_I
	ifa	==, $29,	jp .chip8_FX29_I_eq_sprite_location_of_vx_char
	ifa	==, $33,	jp .chip8_FX33_store_vx_bcd_at_I
	ifa	==, $55,	jp .chip8_FX55_dump_v0_to_vx_at_I
	ifa	==, $65,	jp .chip8_FX65_load_v0_to_vx_at_I
; if none match, show error
	bug_break	"... FX%A% not implemented ERROR !@#$%"
; at this point, C holds [X], HL points to next opcode
.chip8_FX07_vx_eq_delay_timer
	ld	b, c	; move [X] to b
	ld	c, LOW(TIMER_DELAY)
	ld	a, [$FF00+c]	; load delay timer value
	bug_message	"... FX07 vx eq delay timer (%A%)"
	ld	c, b	; [c] points to [X]
	ld	[$FF00+c], a	; set [X] to delay timer value
	jp	chip8.decode_opcode
.chip8_FX0A_vx_eq_key_pressed
	bug_message	"... FX0A WAITING for keypress <^v>"
	push	hl	; store PC
	push	bc	; store b & [X]
.wait4key
	call	get_key_press
	; WAIT for a keypress, then store in vx
	ifa	==, -1, jr .wait4key
	bug_message	"... FX0A vx eq key pressed (%A%)"
	pop	bc	; c points to [X]
	ld	[$FF00+c], a	; set [X] to active key value
	jp	chip8.pop_pc
.chip8_FX15_delay_timer_eq_vx
	ld	a, [$FF00+c]	; load VX
	bug_message	"... FX15 delay timer eq vx (%A%)"
	ld	c, LOW(TIMER_DELAY)	; [c] points to [TIMER_DELAY]
	ld	[$FF00+c], a	; load TIMER_DELAY with VX
	jp	chip8.decode_opcode
.chip8_FX18_sound_timer_eq_vx
	ld	a, [$FF00+c]	; load VX
	bug_message	"... FX18 sound timer eq vx (%A%)"
	ld	c, LOW(TIMER_SOUND)	; [c] points to [TIMER_SOUND]
	ld	[$FF00+c], a	; load TIMER_SOUND with VX
	jp	chip8.decode_opcode
.chip8_FX1E_add_vx_to_I
	ld	a, [$FF00+c]	; load VX
	bug_message	"... FX1E add vx (%A%) to I"
	push	hl	; store PC
	ld	hl, REG.I_LSB
	add	[hl]	; add LSB of I to VX
	ld	[hl], a	; store I_LSB + VX
	jp	nc, chip8.pop_pc	; if no overflow, done
	; if overflow, increment MSB of I
	dec	hl	; CRITICAL: This assumes REG.I == 1 + REG.I_LSB
	; HL now points to REG.I
	inc	[hl]	; increment REG.I's MSB
	jp	chip8.pop_pc
.chip8_FX29_I_eq_sprite_location_of_vx_char
; I equals location of font data pointed at by VX. In this case, we add
; VX to CHIP8_FONT. It's like a lookup table, so we need to scale VX by the
; size of each font (5 bytes). So we must multiply VX * 5 then add to bottom of
; font address to get pointer to correct font (each font is 8x5 pixels)
	ld	a, [$FF00+c]	; load VX
	bug_message	"... FX29 I eq sprite location of vx char (%A%)"
	push	hl		; store PC
	ld	l, a
	xor	a
	ld	h, a
	; HL = $00AA (aka 1*A)
	ldpair	bc, hl	; store A
	add	hl, hl	; HL = 2A
	add	hl, hl	; HL = 4A
	add	hl, bc	; HL = 5A
	; HL is now 5 * VX
	ld	de, CHIP8_FONT	; load location of fonts / graphics
	; we'll add the scaled VX offset to CHIP8_FONT
	add	hl, de	; HL now equals address of vx char
	bug_message	"... FX29 I == %HL%"
	store_rpair_from_hl	REG.I	; overwrites A, C, HL
	jp	chip8.pop_pc
.chip8_FX33_store_vx_bcd_at_I
	; C holds [X]. HL points to next opcode
	push	hl	; preserve PC
	ld	a, [$FF00+c]	; get VX
	bug_message	"... FX33 store vx (%A%) bcd at I"
	; convert binary value in A to unpacked BCD form into ABC, MSB->LSB
	ld	l, a
	xor	a
	ld	h, a	; HL = $00AA, where AA = value in A
	ld	b, h
	ld	c, l	; copy HL to BC
	add	hl, hl	; 2x HL
	add	hl, bc	; 3x HL
	; we now have 3*A stored in HL
	ld	bc, .FX33_bcd_table
	add	hl, bc	; add bcd_table address to our 3*A.
	ldpair	bc, hl	; move bcd address (+ offset) to BC
	; BC now points to the start of BCD representation of A
	load_rpair_into_hl	REG.I	; overwrite A, HL
	; HL now points to where REG.I points
	ld	a, [bc]	; load HUNDREDS bcd digit
	bug_message	"... 100's digit (%A%) stored in %HL%"
	ldi	[hl], a	; store hundreds digit at I, advance I pointer
	inc	bc	; bc now points to tens digit
	ld	a, [bc]	; load TENS bcd digit
	bug_message	"... 10's digit (%A%) stored in %HL%"
	ldi	[hl], a	; store tens digit at I+1, advance to I pointer
	inc	bc	; bc now points to ones digit
	ld	a, [bc]	; load ONES bcd digit
	bug_message	"... 1's digit (%A%) stored in %HL%"
	ldi	[hl], a	; store ones digit at I+2. I is +3'd now
	; CRITICAL WARNING
	; Do we update REG.I, or not?? Wiki doesn't say
	; Based on seeing source, we do NOT update reg.I
	jp	chip8.pop_pc
.FX33_bcd_table
; table for conversion between binary # and 3 bytes of BCD, where the order of
; digits is HUNDREDS, TENS, ONES digits
NUMBER = -1
	REPT 255
NUMBER = NUMBER + 1
HUNDREDS = NUMBER / 100
TENS = (NUMBER % 100) / 10
ONES = NUMBER % 10
	DB	HUNDREDS
	DB	TENS
	DB	ONES
	ENDR
;end bcd_table
.chip8_FX55_dump_v0_to_vx_at_I
; c holds [X]
	push	hl	; preserve PC
	load_rpair_into_hl	REG.I
	bug_message	"... FX55 dump v0 to vx at I (starting at %HL%)"
	ld	a, c	; a holds [X]
	ld	c, LOW(REG.0)	; c is [REG.0]
	sub	c	; A now holds [X] - [0] aka # of registers to copy
	ld	b, a	; b holds # of registers
	; register B is now effectively a counter, since we copy from registers
	; 0-X. If X == 0, we copy 1 register (0). If X == 1, we copy 2
	; registers. (0-1)
	inc	b	; so we increment B (our counter) before beginning
._FX55_dump_loop
	ld	a, [$FF00+c]	; load Vy
	ldi	[hl], a		; write Vy to I, then increment I
	inc	c	; increment to next register
._FX55_check_done
	dec	b	; check if we have more registers to copy
	jr	nz, ._FX55_dump_loop
._FX55_done
	bug_message	"... FX55 dump ended at %HL%"
	;store_rpair_from_hl	REG.I	; store I
	jp	chip8.pop_pc
.chip8_FX65_load_v0_to_vx_at_I
; c holds [X]
	push	hl	; preserve PC
	load_rpair_into_hl	REG.I	; overwrites A, HL
	bug_message	"... FX65 load v0 to vx at I (starting at %HL%)"
	ld	a, c	; A is [X]
	ld	c, LOW(REG.0)	; c is [REG.0]
	sub	c	; A = REG.X - REG.0 (aka A = # of registers to copy)
	ld	b, a	; B is difference in registers
	; register B is now effectively a counter, since we copy from I to
	; registers 0-X. If X == 0, we copy to 1 register (0). If X == 1, we
	; copy to 2 registers (0-1)
	inc	b	; so we increment B (our counter) before beginning
._FX65_restore_loop
; DO WE INCREMENT I??? Wiki has ambiguous wording
; but I think we do, since this is supposed to mimic a load from stack
	ldi	a, [hl]	; read register Vy, then increment I
	ld	[$FF00+c], a	; write Vy
	inc	c	; increment to next register
._FX65_check_done
	dec	b	; check if we have more registers to copy to
	jr	nz, ._FX65_restore_loop
._FX65_done
	bug_message	"... FX65 load ended with I at %HL%"
	;store_rpair_from_hl	REG.I	; store I
	jp	chip8.pop_pc


; decrement TIMERs until they reach 0
vblank_handle_timers:
	push	AF
	ldh	a, [TIMER_DELAY]
	or	a
	jr	z, .skip_delay_decrement
	dec	a
	bug_message	"delay timer--  => %A%"
	ldh	[TIMER_DELAY], a
.skip_delay_decrement
	ldh	a, [TIMER_SOUND]
	or	a
	jr	z, .nosound
	dec	a
	ldh	[TIMER_SOUND], a
	; should play a buzzer sound here, since TIMER_SOUND was nonzero
	ld	a, %00010100
	ldh	[rNR10], a
	ld	a, $96
	ldh	[rNR11], a
	ld	a, $73
	ldh	[rNR12], a
	ld	a, $11
	ldh	[rNR13], a
	ld	a, $85
	ldh	[rNR14], a
.nosound
.done
	pop	AF
	reti

; create graphic characters (5 rows of 4 pixels each) for use in first portion
; of chip8 rom
generate_0_F_gfx: MACRO
	PUSHO

	OPT	b *	; map  (space) and *(star) to 0, 1, respectively
; 0
	DB	%****    
	DB	%*  *    
	DB	%*  *    
	DB	%*  *    
	DB	%****    
; 1
	DB	%  *     
	DB	% **     
	DB	%  *     
	DB	%  *     
	DB	% ***    
; 2
	DB	%****    
	DB	%   *    
	DB	%****    
	DB	%*       
	DB	%****    
; 3
	DB	%****    
	DB	%   *    
	DB	%****    
	DB	%   *    
	DB	%****    
; 4
	DB	%*  *    
	DB	%*  *    
	DB	%****    
	DB	%   *    
	DB	%   *    
; 5
	DB	%****    
	DB	%*       
	DB	%****    
	DB	%   *    
	DB	%****    
; 6
	DB	%****    
	DB	%*       
	DB	%****    
	DB	%*  *    
	DB	%****    
; 7
	DB	%****    
	DB	%   *    
	DB	%  *     
	DB	% *      
	DB	% *      
; 8
	DB	%****    
	DB	%*  *    
	DB	%****    
	DB	%*  *    
	DB	%****    
; 9
	DB	%****    
	DB	%*  *    
	DB	%****    
	DB	%   *    
	DB	%****    
; A
	DB	%****    
	DB	%*  *    
	DB	%****    
	DB	%*  *    
	DB	%*  *    
; B
	DB	%***     
	DB	%*  *    
	DB	%***     
	DB	%*  *    
	DB	%***     
; C
	DB	%****    
	DB	%*       
	DB	%*       
	DB	%*       
	DB	%****    
; D
	DB	%***     
	DB	%*  *    
	DB	%*  *    
	DB	%*  *    
	DB	%***     
; E
	DB	%****    
	DB	%*       
	DB	%****    
	DB	%*       
	DB	%****    
; F
	DB	%****    
	DB	%*       
	DB	%****    
	DB	%*       
	DB	%*       

	POPO
	ENDM

hex_gfx_data:
	generate_0_F_gfx
hex_gfx_data_end:


rom_pong:
  incbin	"brix.rom"
.end
.keypad
		; Down, Up, Left, Right, Start, Select, B, A
	DB	  -1,   -1, 4,    6,     -1,    -1,     -1,-1

rom_pong2:
	DB	$00,$E0	; erase screen
	DB	$60,5	; load V0 (X) with 5
	DB	$61,6	; load V1 (Y) with 6
	; loaded x,y so that drawn characters span multiple tiles
	DB	$62,$0F	; load V2 (character) with F
	DB	$F2,$29	; set index = REG.2's char
	DB	$D0,$15	; draw character for N-length
	; now letters down to A
	DB	$72,$FF	; V2 += 255 (aka -1) = char E
	DB	$F2,$29	; set index = REG.2's char
	DB	$70,7	; move X coordinate to the right by 7
	DB	$D0,$15	; draw E after the F
	; draw another letter
	DB	$72,$FF	; V2 += 255 (aka -1) = char D
	DB	$F2,$29	; set index = REG.2's char
	DB	$70,7	; move X coordinate to the right
	DB	$D0,$15	; draw next letter
	; letters F E D should be drawn on screen
	DB	$F2,$0A	; wait for a keypress, store in V2
	DB	$70,7	; move X coordinate to the right
	DB	$F2,$29	; set index = REG.2's char
	DB	$D0,$15	; draw letter being pressed
	; [$224]
	DB	$E2,$9E	; skip next instruction if key in REG.2 is pressed
	DB	$12,$00	; RESET. Jump to address $200. Aka re-run program
	DB	$12,$24	; jump to address $224

rom_pong2_end:


