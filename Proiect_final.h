//initializare

#include <avr/interrupt.h>

#define TRUE 1u
#define FALSE 0u

#define treisec 11718
#define jumasec 1953

#define SET_BIT( bit_name )		( 1 << bit_name )
#define CLEAR_BIT( bit_name )	( ~(SET_BIT(bit_name)))

/* motoare */
#define ENA PORTD6		/* COM0A1 */
#define IN1 PORTD3
#define IN2 PORTD4
#define IN3 PORTD7
#define IN4 PORTD2		
#define ENB PORTD5		/* COM0B1 */

/* senzor de linie */
#define S1 PORTC0		
#define S2 PORTC1
#define S3 PORTC2
#define S4 PORTC3
#define S5 PORTC4

/* senzor de culoare */
#define S00 PORTB0
#define S01 PORTB1
#define S02 PORTB2
#define S03 PORTB3
#define OUT PORTB4

#define rosu		1u
#define albastru	2u
#define verde	3u

/* sonar */
#define trigger PORTC5
#define echo PORTB5

/* Regulator PID */
#define kp 1u
#define kd 1u
#define ki 1u
#define dt 1u
#define max_pwm 255u
#define min_pwm 0u


typedef unsigned char uint8;
typedef signed char sint8;

void init( void );
void init_PWM_timer0( void ); //pentru motoare
void init_PWM_timer1( void ); //sonar
void init_PWM_timer2( void );  //delay

extern void INIT( void );
extern uint8 PID( uint8, uint8 );
extern void motor_reglat( void );
extern uint8 culoare( void );
void initializare_senzor_culoare( uint8 , uint8 );
extern float sonar( void );
extern void robot( void );
void delay( int );


void initializare_senzor_culoare( uint8 frequency1, uint8 frequency2 )
{
	if( ( frequency1 != FALSE) && ( frequency2 != FALSE ) )		/* 100% output frequency scaling */
	{
		DDRB |= SET_BIT( S00 ) | SET_BIT( S01 );
	}
	else if ( ( frequency1 != FALSE ) && ( frequency2 == FALSE ) )		/* 20% output frequency scaling */
	{
		DDRB |= SET_BIT( S00 ) & CLEAR_BIT( S01 );
	}
	else if ( ( frequency1 == FALSE ) && ( frequency2 != FALSE ) )		/* 2% output frequency scaling */
	{
		DDRB &= CLEAR_BIT( S00 ) | SET_BIT( S01 );
	}
	else if ( ( frequency1 == FALSE ) && ( frequency2 == FALSE ) )		/* Power Down */
	{
		DDRB &= CLEAR_BIT( S00 ) & CLEAR_BIT( S01 );
	}
}
void init_ports( void )
{
	DDRD |= SET_BIT( ENA ) | SET_BIT( IN1 ) | SET_BIT( IN2 ) | SET_BIT( IN3 ) | SET_BIT( IN4 ) | SET_BIT( ENB );
	DDRC &= CLEAR_BIT( S1 ) & CLEAR_BIT( S2 ) & CLEAR_BIT( S3 ) & CLEAR_BIT( S4 ) & CLEAR_BIT( S5 );
	DDRB |= ( SET_BIT ( S00 ) | SET_BIT ( S01 ) | SET_BIT ( S02 ) | SET_BIT ( S03 ) )  & CLEAR_BIT ( OUT );
}

void init_PWM_timer0( void ) 
{
	cli();
	TCCR0A = TCCR0B = 0;
	TCCR0A |= SET_BIT( WGM01 ) | SET_BIT( WGM00 ) | SET_BIT( COM0A1 ) | SET_BIT( COM0B1 );
	TCCR0B |= SET_BIT( CS00 );
	sei();
}

void init_PWM_timer1( void ) 
{
	cli();
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B = (1 << CS10);                           // FARA prescaler
	sei();
}

void init_PWM_timer2( void )
{
	cli();
	TCCR2A=0;
	TCCR2B=0;
	TCCR2B|=(1<<CS22)|(1<<CS21)|(1<<CS20);//prescaler 1024
	sei();
}

void delay ( int wait )
{
	int tact=0;
	TCNT2=0;
	while( tact <= wait )
	{
		if( TCNT2 == 255 )
		{
			TCNT2=0;
			tact++;
		}
		TCNT2++;
	}
}

void INIT( void )
{
	init_ports();
	init_PWM_timer0();
	init_PWM_timer1();
	init_PWM_timer2();
	initializare_senzor_culoare( TRUE, FALSE );
}

//motoare

extern void PWM( int, int, int );

void PWM(int timeON_A, int timeON_B, int stare) 
{
	PORTD &= CLEAR_BIT( ENA ) & CLEAR_BIT( IN1 ) & CLEAR_BIT( IN2 ) & CLEAR_BIT( IN3 ) & CLEAR_BIT( IN4 ) & CLEAR_BIT( ENB );
	
	if( stare == 0u )	/* frana */
	{
		if( timeON_A == 0u && timeON_B == 0u ) 
		{
			PORTD |= SET_BIT  ( ENA );			
			PORTD &= CLEAR_BIT( IN1 );
			PORTD &= CLEAR_BIT( IN2 );
			PORTD &= CLEAR_BIT( IN3 );
			PORTD &= CLEAR_BIT( IN4 );
			PORTD |= SET_BIT  ( ENB );
			OCR0A = timeON_A;
			OCR0B = timeON_B;
		}
	}
	else if( stare == 1u )	/* stanga */
	{
		if( timeON_A > 0u && timeON_B > 0u ) 
		{
			PORTD |= SET_BIT  ( ENA );
			PORTD |= SET_BIT  ( IN1 );
			PORTD &= CLEAR_BIT( IN2 );
			PORTD &= CLEAR_BIT( IN3 );
			PORTD |= SET_BIT  ( IN4 );
			PORTD |= SET_BIT  ( ENB );
			OCR0A = timeON_A;
			OCR0B = timeON_B;
		}
	}
	else if( stare == 2u )	/* dreapta */
	{
		if( timeON_A > 0u && timeON_B > 0u ) 
		{
			PORTD |= SET_BIT  ( ENA );
			PORTD &= CLEAR_BIT( IN1 );
			PORTD |= SET_BIT  ( IN2 );
			PORTD |= SET_BIT  ( IN3 );
			PORTD &= CLEAR_BIT( IN4 );
			PORTD |= SET_BIT  ( ENB );
			OCR0A = timeON_A;
			OCR0B = timeON_B;
		}
	}
	else if( stare == 3u )	/*spate*/
	{
		if( timeON_A > 0u && timeON_B > 0u ) 
		{
			PORTD |= SET_BIT  ( ENA );
			PORTD |= SET_BIT  ( IN1 );
			PORTD &= CLEAR_BIT( IN2 );
			PORTD |= SET_BIT  ( IN3 );
			PORTD &= CLEAR_BIT( IN4 );
			PORTD |= SET_BIT  ( ENB );
			OCR0A = timeON_A;
			OCR0B = timeON_B;
		}
	}
	else if( stare == 4u )	/* fata */
	{
		if( timeON_A > 0u && timeON_B > 0u ) 
		{
			PORTD |= SET_BIT  ( ENA );
			PORTD &= CLEAR_BIT( IN1 );
			PORTD |= SET_BIT  ( IN2 );
			PORTD &= CLEAR_BIT( IN3 );
			PORTD |= SET_BIT  ( IN4 );
			PORTD |= SET_BIT  ( ENB );
			OCR0A = timeON_A;
			OCR0B = timeON_B;
		}
	}
}

//PID

sint8 eroarea_anterioara = 0;

uint8 PID( uint8 viteza_curenta, uint8 viteza_anterioara )
{
	sint8 eroare;
	sint8 iesire_proportional;
	sint8 integral = 0;
	sint8 iesire_integrator;
	sint8 derivator;
	sint8 iesirea_derivator;
	sint8 iesirea;

	/* calcularea erorii */
	eroare = ( sint8 )viteza_curenta - ( sint8 )viteza_anterioara;
	
	/* Componenta Proportionala */
	iesire_proportional = ( sint8 )kp * eroare;
	
	/* Componenta Integratoare */ 
	integral += eroare * ( sint8 )dt;
	iesire_integrator = ki * integral;
	
	/* Componenta Derivatoare */
	derivator = ( eroare - eroarea_anterioara ) / ( sint8 )dt;
	iesirea_derivator = ( sint8 )kd * derivator;
	
	/* Calculul Iesirii */
	iesirea = iesire_proportional + iesire_integrator + iesirea_derivator;
	
	/* verificare limita inferioara si superioara nedepasita */
	if( iesirea > ( sint8 )max_pwm )
	{
		iesirea = max_pwm;
	}
	else if( iesirea < ( sint8 )min_pwm )
	{
		iesirea = min_pwm;
	}
	
	/* salveaza noua eroare */
	eroarea_anterioara = eroare;
	
	/* returneaza iesirea reglata */
	return ( uint8 )iesirea;
}

//robot

void robot( void )
{

	motor_reglat();
	if( culoare() == rosu )
	{
		PWM( FALSE, FALSE, FALSE );
		delay( treisec );
	}
	else if( culoare() == albastru )
	{
		motor_reglat();
	}
	else if( culoare() == verde )
	{
		PWM( PID( 100u, 255u ), PID( 100u, 255u ), 2u );
		motor_reglat();
	}
	if( sonar() < 25 )
	{
		PWM( FALSE, FALSE, FALSE );
		delay( jumasec );
		PWM( PID( 200u, 255u ), PID( 200u, 255u ), 1u );
		delay( jumasec );
		PWM( PID( 200u, 200u ), PID( 200u, 200u ), 4u );
		delay( jumasec );
		PWM( PID( 200u, 200u ), PID( 200u, 200u ), 2u );
		delay( jumasec );
		PWM( PID( 200u, 200u ), PID( 200u, 200u ), 4u );
		delay( jumasec );
		PWM( PID( 200u, 200u ), PID( 200u, 200u ), 2u );
		delay( jumasec );
		PWM( PID( 200u, 200u ), PID( 200u, 200u ), 4u );
		delay( jumasec );
		PWM( PID( 200u, 255u ), PID( 200u, 200u ), 1u );
		motor_reglat();
	}	
}

//senzor_culoare

uint8 culoare( void )
{
	uint8 culoare = FALSE;
	uint8 photodiode_type1 = FALSE;
	uint8 photodiode_type2 = FALSE;
	if( ( photodiode_type1 == FALSE ) && ( photodiode_type2 == FALSE ) )
	{
		DDRB &= CLEAR_BIT( S02 ) & CLEAR_BIT( S03 );
		culoare = RED;
	}
	else if( ( photodiode_type1 == FALSE ) && ( photodiode_type2 != FALSE ) )
	{
		DDRB &= CLEAR_BIT( S02 ) | SET_BIT( S03 );
		culoare = BLUE;
	}
	else if( ( photodiode_type1 != FALSE ) && ( photodiode_type2 != FALSE ) )
	{
		DDRB |= SET_BIT( S02 ) | SET_BIT( S03 );
		culoare = GREEN;
	}
	
	return culoare;
}

//senzor linie

void motor_reglat( void )
{
	/* fata */
	while( ( PINC | ( SET_BIT( S3 ) ) ) != FALSE )
	{
		PWM( PID( 255u, 0u ), PID( 255u, 0u ), 4u );
	}
	
	/* stanga curba usoara */
	while( ( PINC | ( SET_BIT( S2 ) ) ) != FALSE )
	{
		PWM( PID( 200u, 255u ), PID( 200u, 255u ), 1u );
	}
	
	/* stanga curba brusca */
	while( ( PINC | ( SET_BIT( S1 ) ) ) != FALSE )
	{
		PWM( PID( 100u, 255u ), PID( 100u, 255u ), 1u );
	}
	
	/* dreapta curba usoara */
	while( ( PINC | ( SET_BIT( S4 ) ) ) != FALSE )
	{
		PWM( PID( 200u, 255u ), PID( 200u, 255u ), 2u );
	}
	
	/* dreapta curba brusca */
	while( ( PINC | ( SET_BIT( S5 ) ) ) != FALSE )
	{
		PWM( PID( 100u, 255u ), PID( 100u, 255u ), 2u );
	}
}


//sonar

float sonar( void )
{
	float timp, distanta;
	int overflow = 0;                                        // CONTOR OVERFLOW

	TCNT1 = 0;
	PORTC &= CLEAR_BIT( trigger );
	while( TCNT1 <= 160 )                               // 160 < = > 10 microsecunde
	TCNT1++;
	PORTC |= SET_BIT( trigger );
	TCNT1 = 0;
	while( TCNT1 <= 160 )
	TCNT1++;
	PORTC &= CLEAR_BIT( trigger );
	TCNT1 = 0;
	while(PINB & SET_BIT( echo ) )	//echo
	{                              
		TCNT1++;
		if( TCNT1 == 65535 ) 
		{
			overflow++;
			TCNT1 = 0;
		}
	}

	timp = ( ( overflow * 65535 ) + TCNT1 ) * 0.125;
	distanta = timp / 58;
	
	return distanta;
}