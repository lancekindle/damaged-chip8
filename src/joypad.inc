; these are methods for joypads taken from gameboy examples

include "src/gbhw.inc"
include "src/syntax.inc"

	IF  !DEF(JOYPAD_INC)
JOYPAD_INC  SET     1

; gbhw.inc defines the down, up, left, right, start, select, B, etc. as...
; PADF_DOWN, PADF_UP, PADF_LEFT, PADF_RIGHT, PADF_START, PADF_SELECT, PADF_B..

jpad_dpad_mask	SET	PADF_UP | PADF_DOWN | PADF_LEFT | PADF_RIGHT
; define our own joypad variables for keeping track of when a button was last
; pressed


; IF (!DEF(jpad_rKeys)) | !DEF(jpad_rHexEncoded)) */
; 	FAIL "please define jpad_rKeys and jpad_rHexEncoded as RAM addresses before calling jpad_GetKeys" */
; ENDC

; gets currently pressed keys. Register A will hold keys in the following
; order: MSB --> LSB (Most Significant Bit --> Least Significant Bit)
; Down, Up, Left, Right, Start, Select, B, A
; Register B will hold values representing which buttons were JUST pressed
; in the same order as Register A
; \1 (Argument 1) is register from which to read and write active keys
; \2 (argument 2) is memory address from which to write single active key
; because chip8 can only accept 1 keypress at a time
jpad_GetKeys: MACRO
	; USES: AF, BC, HL
	; variables JOYPAD_BUTTONS and JOYPAD_ARROWS are defined in gbhw.inc
	; get action buttons: A, B, Start / Select
	ld	a, JOYPAD_BUTTONS; choose bit that'll give us action button info
	ld	[rJOYPAD], a; write to joypad, telling it we'd like button info
	ld	a, [rJOYPAD]; gameboy will write (back in address) joypad info
	ld	a, [rJOYPAD]
	cpl		; take compliment
	and	$0f	; look at first 4 bits only  (lower nibble)
	swap	a	; place lower nibble into upper nibble
	ld	b, a	; store keys in b
	; get directional keys
	ld	a, JOYPAD_ARROWS
	ld	[rJOYPAD], a ; write to joypad, selecting direction keys
	ld	a, [rJOYPAD]
	ld	a, [rJOYPAD]
	ld	a, [rJOYPAD]
	ld	a, [rJOYPAD]
	ld	a, [rJOYPAD]
	ld	a, [rJOYPAD]	; delay to reliably read keys
	cpl			; take compliment
	and	$0f		; keep lower nibble
	or	b		; combine action & direction keys (result in a)

	; now keep current keys if still pressed. Otherwise use new keys
	ld	b, a			; store current keys in b
	ld	a, [jpad_rKeys]		; load keys from last time
	ld	c, a			; store old keys in c
	ld	a, b
	ld	[jpad_rKeys], a		; store new keys in ram
	ld	a, c			; restore old keys in a
.check_if_no_keys_pressed_previously\@
	; if no keys pressed previously, automatically skip the "old keys still
	; pressed" check, since it would pass when no keys are pressed
	and	a
	jr	z, .new_keys_pressed\@
	; if new keys include previously pressed keys, continue to use old keys
	; (using this syntax to account for modifier + key)
.check_if_old_keys_still_pressed\@
	; if all the old keys are still pressed, continue using old result
	ld	c, a	; store old keys in C
	; A == C right now. If after ANDing with B (the new keys) A == C still,
	; then we know that the old keys are still being actively pressed
	and	b
	ifa	==, c, jp .use_old_keys\@
.new_keys_pressed\@
	; we get here if old keys not fully pressed. Need to trim to 1 key
	ld	a, b	; restore new keys in A
	; old keys in C
	or	c
	xor	c	; adds then removes previously pressed keys
	; what remains are newly pressed keys
	; UNLESS ONLY A KEY WAS RELEASED
	; then Z flag will be set, indicating to just use new keys
	if_flag	z,	ld	a, b	; restore new keys if (new - old = none)
	; hex keys are 0-F.
.find_single_key\@
	; bits are represented as
	; Down, Up, Left, Right, Start, Select, B, A
	; Stop when first active key found. That'll be used as only active key
	ld	b, 0
	REPT	8
		rlca	; rotate a to left (copying the floating bit from bit7 to CY and bit0)
		jr	c, .store_active_key	; found a pressed key
		inc	b
	ENDR
.no_keys_pressed\@
	ld	a, -1
	jr	.store_hex\@
.store_active_key	; WARNING we can't have a \@ at end, so don't re-call this macro
	; we get here if we found an active key! B holds the active key
	; represented as 0-7:
	; D, U, L, R,     Start,Select,   B, A
	; So Down == 0, Up == 1, etc..., B == 6, A == 7
	ld	a, b	; active key (represented as 
	; store active key in jpad_rActiveKey
	ld	[jpad_rActiveKey], a	; store single active key
.get_mapped_value\@
	ld	hl, keypad_map
	; adjust keypad_map by a (0-16)
	add	l
	ld	l, a
	if_flag	c,	inc h
	ld	a, [hl]	; load remapped key value for active key
.store_hex\@
	; A contains hex value of keypress
	ld	[jpad_rHexEncoded], a
	jr	.done\@
.use_old_keys\@
	ld	a, [jpad_rHexEncoded]
	; old keys are still pressed
	jr	.done\@
.done\@
	ENDM

        ENDC  ; end defining JOYPAD STUFF
