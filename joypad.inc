; these are methods for joypads taken from gameboy examples

include "gbhw.inc"
include "syntax.inc"

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
	; USES: AF, B
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
	; we get here if old keys not fully pressed. (maybe a modifier or key
	; has been released). Use new key + modifier. Need to trim to 1 mod+key
	ld	a, b	; restore new keys in A
	and	PADF_START | PADF_SELECT	; modifier keys mask
	ifa	==, b, jp .only_modifiers_pressed\@
	; above check also works if no keys pressed. In both cases, return -1
	; hex keys are 0-F. Since only arrows + a + b are used, the start and
	; select keys modify the key mapping to give us 3x 6-keys = 18.
	ld	c, 0
	; set corresponding bit in C equal to modifer bit in A
	if_bit	PADB_SELECT, a, ld c, PADF_SELECT
	if_bit	PADB_START, a, ld c, PADF_START	; start bit will overwrite if it's active
.elimate_all_but_1_key\@
	xor	b	; remove modifer keys from active keys. Active keys 
			; without modifiers now in A
			; C holds hex offset for the chosen active modifier
	; next up we eliminate all but one active key
	ld	b, %00000001	; we'll use this as the active key
	; register B will be rotated in the opposite direction as A such that
	; when A finally rotates a '1' into CY, B will have a '1' in the bit
	; position where the '1' in CY originally started within A
.find_single_key_loop\@
	rrc	b	; rotate b to right  (on first loop it'll rotate into bit 7)
	rlca	; rotate a to left (copying the floating bit from bit7 to CY and bit0)
	jr	nc, .find_single_key_loop\@	; keep looping until CY=1
	; we get here if we found an active key! B holds the active key bit
	ld	a, b	; boom. Single active key found, loaded in A.
.store_active_keys\@
	; store active key + modifier in jpad_rKeys
	xor	c	; add in modifier key
	ld	[jpad_rKeys], a	; store single active key + single modifer key
	xor	c	; remove modifier key
.encode_keypress_as_hex\@
	; bits are represented as
	; Down, Up, Left, Right, Start, Select, B, A
	; encode the pressed key + modifier as 0-F
	; 16 is minimum value for any directional key pressed. Mask = %11110000
	ifa	>=, 16, jr .direction_keys_pressed\@
.buttons_pressed\@
	; both A & B contain the same key
	; depending on modifiers, values are in range $0A-$0F
	if_bit	0, b,	ld a, $0A
	if_bit	1, b,	ld a, $0B
	; now add modifier key (only 1 modifier is active)
	if_bit	0, c,	add 2	; $0A -> $0C. $0B -> $0D
	if_bit	1, c,	add 4	; $0A -> $0E, $0B -> $0F
.direction_keys_pressed\@
	; both A & B contain the same key
	if_bit	7, b,	ld a, $02  ; down key
	if_bit	6, b,	ld a, $08  ; up key
	if_bit	5, b,	ld a, $04  ; left key
	if_bit	4, b,	ld a, $06  ; right key
	; now add modifier keys
	if_bit	0, c,	sub 1	; get $01, $07, $03, or $05
	if_not_bit	1, c,	jr .store_hex\@
	; we get here if bit 1 of C is set
	sub	2	; $00, $06, $03, $05
	ifa	>, 0,	add 6	; $00, $0C, $09, $0B
.store_hex\@
	; A contains hex value of keypress
	ld	[jpad_rHexEncoded], a
	jr	.done\@
.use_old_keys\@
	ld	a, [jpad_rHexEncoded]
	; old keys are still pressed
	jr	.done\@
.only_modifiers_pressed\@
	; store 0 in rKeys variable. CRITICAL. If we didn't, then the modifier
	; key would count as the active key, and the above check for
	; new_keys_pressed would use the old keys (just the modifiers)
	xor	a
	ld	[jpad_rKeys], a	; store 0 as pressed keys. CRITICAL
	ld	a, -1	; load -1 aka NO KEY representation
	ld	[jpad_rHexEncoded], a
.done\@
	ld	b, a	; store backup of hex value
.reset_joypad
	; reset joypad port. Allows interrupt triggering if enabled.
	xor	a
	ld	a, P1F_5|P1F_4  ; JOYPAD_BUTTONS | JOYPAD_ARROWS
	ld	[rP1], a	; rJOYPAD	; reset joypad
.end_with_hex_value\@
	ld	a, b	; restore hex value in a
	ENDM

        ENDC  ; end defining JOYPAD STUFF
