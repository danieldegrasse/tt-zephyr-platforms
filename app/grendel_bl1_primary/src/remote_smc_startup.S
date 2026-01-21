.global _start
_start:
	mv a0, x0
	mv a1, x0
	# Set a0 to 0xc0010108 (scratch register 2)
	lui a0, 0xc001
	slli a0, a0, 0x4
	addi a0, a0, 0x110
	# Note that sometimes "T" might be printed, depending on the state of
	# the TOGGLE bit in the testbench.
	# Store 0x5401 to scratch reg. This results in a print to of "T" to console.
	li a1, 0x5401
	sw a1, (a0)
	# Store 0x5400 to scratch reg. This results in a print to of "T" to console.
	li a1, 0x5400
	sw a1, (a0)
	# Store 0xa01 to scratch reg. This results in a print of "\n" to console.
	li a1, 0xa01
	sw a1, (a0)
_loop:
	# Spin here
	j _loop
