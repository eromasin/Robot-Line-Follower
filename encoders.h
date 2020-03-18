#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26

volatile long count_right_e; // used by encoder to count the rotation
volatile bool oldE1_A;  // used by encoder to remember prior state of A
volatile bool oldE1_B;  // used by encoder to remember prior state of B
volatile long count_left_e; // used by encoder to count the rotation
volatile bool oldE0_A;  // used by encoder to remember prior state of A
volatile bool oldE0_B;  // used by encoder to remember prior state of B

ISR( INT6_vect ) {
  boolean newE1_B = digitalRead( E1_B_PIN );
  boolean newE1_A = digitalRead( E1_A_PIN );
  newE1_A ^= newE1_B;
  byte state = 0;
  state = state | ( newE1_A  << 3 );
  state = state | ( newE1_B  << 2 );
  state = state | ( oldE1_A  << 1 );
  state = state | ( oldE1_B  << 0 );
  switch( state ) {
    case 0:                   break; // No movement.
    case 1:  count_right_e--; break;  // clockwise?
    case 2:  count_right_e++; break;  // anti-clockwise?
    case 3:                   break;  // Invalid
    case 4:  count_right_e++; break;  // anti-clockwise?
    case 5:                   break;  // No movement.
    case 6:                   break;  // Invalid
    case 7:  count_right_e--; break;  // clockwise?
    case 8:  count_right_e--; break;  // clockwise?
    case 9:                   break;  // Invalid
    case 10:                  break;  // No movement.
    case 11: count_right_e++; break;  // anti-clockwise?
    case 12:                  break;  // Invalid
    case 13: count_right_e++; break;  // anti-clockwise?
    case 14: count_right_e--; break;  // clockwise?
    case 15:                  break;  // No movement.
  }
  oldE1_A = newE1_A;
  oldE1_B = newE1_B;
}

ISR( PCINT0_vect ) {
  boolean newE0_B = PINE & (1<<PINE2);
  boolean newE0_A = digitalRead( E0_A_PIN ); // 26 the same as A8
  newE0_A ^= newE0_B;
  byte state = 0;                   
  state = state | ( newE0_A  << 3 );
  state = state | ( newE0_B  << 2 );
  state = state | ( oldE0_A  << 1 );
  state = state | ( oldE0_B  << 0 );
  switch( state ) {
    case 0:                  break; // No movement.
    case 1:  count_left_e--; break;  // clockwise?
    case 2:  count_left_e++; break;  // anti-clockwise?
    case 3:                  break;  // Invalid
    case 4:  count_left_e++; break;  // anti-clockwise?
    case 5:                  break;  // No movement.
    case 6:                  break;  // Invalid
    case 7:  count_left_e--; break;  // clockwise?
    case 8:  count_left_e--; break;  // clockwise?
    case 9:                  break;  // Invalid
    case 10:                 break;  // No movement.
    case 11: count_left_e++; break;  // anti-clockwise?
    case 12:                 break;  // Invalid
    case 13: count_left_e++; break;  // anti-clockwise?
    case 14: count_left_e--; break;  // clockwise?
    case 15:                 break;  // No movement.
  }
  oldE0_A = newE0_A;
  oldE0_B = newE0_B; 
}

void setupRightEncoder() {
  count_right_e = 0;
  oldE1_A = 0;
  oldE1_B = 0;
  pinMode( E1_A_PIN, INPUT );
  pinMode( E1_B_PIN, INPUT );
  EIMSK = EIMSK & ~(1<<INT6);
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  EIFR |= ( 1 << INTF6 );
  EIMSK |= ( 1 << INT6 );
}

void setupLeftEncoder() {
    count_left_e = 0;
    oldE0_A = 0;
    oldE0_B = 0;
    DDRE = DDRE & ~(1<<DDE6);
    PORTE = PORTE | (1<< PORTE2 );
    pinMode( E0_A_PIN, INPUT );
    digitalWrite( E0_A_PIN, HIGH ); // Encoder 0 xor
    PCICR = PCICR & ~( 1 << PCIE0 );
    PCMSK0 |= (1 << PCINT4);
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.
    PCICR |= (1 << PCIE0);
}
