sub r0, r0
sub r1, r1
sub r2, r2

ori 1
add r2, r1


sub r1, r1
ori 22
shift l r1, 3
store r0, r1
add r0, r2

j -6