sub r0, r0
sub r1, r1
sub r2, r2

ori 1
add r2, r0

sub r0, r0

ori 22
shift l r0, 3

store r1, r0
add r1, r2

j -3